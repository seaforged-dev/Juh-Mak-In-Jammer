#include <Arduino.h>
#include "false_positive.h"
#include "rf_modes.h"   // for rfGetPower()
#include "protocol_params.h"

// ============================================================
// False Positive Generator — LoRaWAN, ISM bursts, and Mixed
// ============================================================

static SX1262 *_radio = nullptr;
static FpMode  _fpMode = FP_LORAWAN;
static bool    _fpRunning = false;

// Counters
static uint32_t _loraCount = 0;
static uint32_t _burstCount = 0;
static uint32_t _mixedElrsCount = 0;
static float    _lastFreq = 0;
static uint8_t  _lastSF = 7;

// --- Simple PRNG for randomized parameters ---
// xorshift32 — fast, good enough for test signal generation
static uint32_t _rng = 0x12345678;

static uint32_t rngNext() {
    _rng ^= _rng << 13;
    _rng ^= _rng >> 17;
    _rng ^= _rng << 5;
    return _rng;
}

static float rngFloat(float lo, float hi) {
    return lo + (float)(rngNext() % 10000) / 10000.0f * (hi - lo);
}

static uint32_t rngRange(uint32_t lo, uint32_t hi) {
    return lo + (rngNext() % (hi - lo + 1));
}

// ============================================================
// LoRaWAN Simulation
// ============================================================
// Real LoRaWAN: SF7-SF12, BW125, single channel, low duty cycle.
// One transmission every 30-60 seconds — typical IoT sensor.

// LoRaWAN frequencies now from protocol_params.h LORAWAN_US915_SB2[]
static unsigned long _loraNextTxMs = 0;

// Randomized dummy payload (20-50 bytes, mimics sensor data)
static uint8_t _loraBuf[50];

static void lorawanConfigRadio() {
    // Pick a random SF between 7-12 for each transmission
    // Real LoRaWAN devices use ADR which varies SF — v2 §4.1
    _lastSF = (uint8_t)rngRange(7, 12);

    // Pick a random SB2 channel — v2 §4.1 [Ref R7]
    uint8_t chIdx = rngNext() % LORAWAN_US915_SB2_COUNT;
    _lastFreq = LORAWAN_US915_SB2[chIdx];

    _radio->standby();
    _radio->begin(
        _lastFreq,
        125.0,                  // BW 125 kHz (standard LoRaWAN)
        _lastSF,
        5,                      // CR 4/5
        SYNC_WORD_LORAWAN,      // 0x34 public LoRa — v2 §6.3
        rfGetPower(),
        LORAWAN_PREAMBLE_LEN,   // 8 symbols — v2 §6.2
        1.8, false
    );
    _radio->explicitHeader();
}

static void lorawanTransmit() {
    lorawanConfigRadio();

    // Random payload length 20-50 bytes
    uint8_t len = (uint8_t)rngRange(20, 50);
    for (uint8_t i = 0; i < len; i++) {
        _loraBuf[i] = (uint8_t)(rngNext() & 0xFF);
    }

    // _lastFreq was set by lorawanConfigRadio via SB2 channel selection
    _radio->transmit(_loraBuf, len);
    _loraCount++;

    // Schedule next TX in 30-60 seconds
    _loraNextTxMs = millis() + rngRange(30000, 60000);

    Serial.printf("LoRaWAN TX: SF%u BW125 @ %.1f MHz, %u bytes\n",
                  _lastSF, _lastFreq, len);
}

// ============================================================
// ISM Burst Noise
// ============================================================
// Random short FSK bursts at random frequencies in 902-928 MHz.
// Simulates garage door openers, TPMS, weather stations, etc.

static unsigned long _burstNextMs = 0;

static void ismBurstTransmit() {
    float freq = rngFloat(902.0f, 928.0f);
    uint32_t burstLen = rngRange(10, 50);  // ms duration → payload size

    _radio->standby();

    // Configure as FSK — diverse bitrates mimic diverse ISM devices
    float bitrate = rngFloat(1.2f, 50.0f);   // kbps
    float freqDev = rngFloat(5.0f, 25.0f);   // kHz deviation

    _radio->beginFSK(
        freq,
        bitrate,
        freqDev,
        156.2,     // RX bandwidth (doesn't matter for TX, but required)
        rfGetPower(),
        8,         // preamble
        1.8, false
    );

    // Generate random noise payload sized to approximate the burst duration
    // At bitrate kbps, burstLen ms ≈ bitrate * burstLen / 8 bytes
    uint8_t payloadLen = (uint8_t)min((uint32_t)(bitrate * burstLen / 8.0f), (uint32_t)255);
    if (payloadLen < 4) payloadLen = 4;
    uint8_t buf[64];
    uint8_t txLen = min(payloadLen, (uint8_t)64);
    for (uint8_t i = 0; i < txLen; i++) {
        buf[i] = (uint8_t)(rngNext() & 0xFF);
    }

    _lastFreq = freq;
    _radio->transmit(buf, txLen);
    _burstCount++;

    // Next burst in 200-2000 ms (random inter-burst interval)
    _burstNextMs = millis() + rngRange(200, 2000);
}

// ============================================================
// Mixed Mode — LoRaWAN background + ELRS foreground (TDM)
// ============================================================
// The SX1262 can only do one thing at a time, so we alternate:
// - ELRS FHSS runs continuously (200 Hz hops)
// - Every 30-60 seconds, we pause ELRS, send one LoRaWAN packet,
//   then reconfigure back to ELRS and resume hopping.

static bool _mixedElrsConfigured = false;
static unsigned long _mixedLoraNextMs = 0;

// ELRS parameters from protocol_params.h — no local duplicates
static const ElrsDomain&  _mixDom  = ELRS_DOMAINS[ELRS_DOMAIN_FCC915];
static const ElrsAirRate& _mixRate = ELRS_AIR_RATES[ELRS_RATE_200HZ];

// Sequence holds non-sync channels only; sync is interleaved at runtime.
static uint8_t  _mixHopSeq[80];  // sized for largest domain
static uint8_t  _mixSeqLen = 0;  // = numCh - 1 after build
static uint8_t  _mixHopIdx = 0;
static uint8_t  _mixPktsSinceHop = 0;
static uint32_t _mixHopCount = 0;
static unsigned long _mixLastPktUs = 0;
static const uint8_t MIX_ELRS_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };

static void mixBuildHopSequence() {
    uint8_t numCh  = _mixDom.channels;
    uint8_t syncCh = _mixDom.syncChannel;

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < numCh; i++) {
        if (i != syncCh) _mixHopSeq[cnt++] = i;
    }
    _mixSeqLen = cnt;

    uint32_t rng = 0xCAFEBABE;
    for (uint8_t i = cnt - 1; i > 0; i--) {
        rng = rng * ELRS_LCG_MULTIPLIER + ELRS_LCG_INCREMENT;
        uint8_t j = (rng >> 16) % (i + 1);
        uint8_t tmp = _mixHopSeq[i];
        _mixHopSeq[i] = _mixHopSeq[j];
        _mixHopSeq[j] = tmp;
    }
}

static void mixConfigElrs() {
    _radio->standby();
    float freq = elrsChanFreq(_mixDom, _mixHopSeq[_mixHopIdx]);
    _radio->begin(freq, (float)(_mixRate.bwHz / 1000), _mixRate.sf, _mixRate.cr,
                  SYNC_WORD_ELRS, rfGetPower(), _mixRate.preambleLen, 1.8, false);
    // Real ELRS uses implicit header (v2 §3.1.4). MIX_ELRS_PAYLOAD is
    // 8 bytes and _mixRate is fixed at 200 Hz FCC915 (payloadLen=8).
    // The LoRaWAN burst path re-configures the radio via begin() +
    // explicitHeader() from lorawanConfigRadio(), and returning here
    // re-installs the implicit header state for the ELRS side.
    _radio->implicitHeader(_mixRate.payloadLen);
    _mixedElrsConfigured = true;
}

static void mixElrsTx() {
    uint8_t numCh    = _mixDom.channels;
    uint8_t hopEvery = _mixRate.hopInterval;

    if (_mixPktsSinceHop >= hopEvery) {
        _mixPktsSinceHop = 0;
        _mixHopCount++;

        uint8_t nextChan;
        if ((_mixHopCount % numCh) == 0) {
            nextChan = _mixDom.syncChannel;
        } else {
            _mixHopIdx = (uint8_t)((_mixHopIdx + 1) % _mixSeqLen);
            nextChan = _mixHopSeq[_mixHopIdx];
        }

        float freq = elrsChanFreq(_mixDom, nextChan);
        _lastFreq = freq;
        _radio->setFrequency(freq);
    }

    _radio->transmit(MIX_ELRS_PAYLOAD, sizeof(MIX_ELRS_PAYLOAD));
    _mixedElrsCount++;
    _mixPktsSinceHop++;
}

static void mixLorawanBurst() {
    // Pause ELRS, send one LoRaWAN packet, resume ELRS
    _mixedElrsConfigured = false;
    lorawanTransmit();  // reconfigures radio to LoRaWAN, transmits, increments _loraCount
    mixConfigElrs();    // reconfigure back to ELRS

    _mixedLoraNextMs = millis() + rngRange(30000, 60000);
    Serial.println("Mixed: LoRaWAN burst inserted, resuming ELRS");
}

// ============================================================
// Public API
// ============================================================

void fpInit(SX1262 *radio) {
    _radio = radio;
    _fpRunning = false;
    _rng = (uint32_t)(micros() ^ 0xDEAD);  // seed from boot time
}

void fpStart(FpMode mode) {
    if (!_radio) return;

    _fpMode = mode;
    _loraCount = 0;
    _burstCount = 0;
    _mixedElrsCount = 0;
    _lastFreq = 0;
    _lastSF = 7;

    switch (mode) {
    case FP_LORAWAN:
        _loraNextTxMs = millis() + 1000;  // first TX after 1 second
        Serial.println("FP: LoRaWAN simulation started");
        break;

    case FP_ISM_BURST:
        _burstNextMs = millis() + 500;
        Serial.println("FP: ISM burst noise started");
        break;

    case FP_MIXED:
        mixBuildHopSequence();
        _mixHopIdx = 0;
        _mixPktsSinceHop = 0;
        _mixHopCount = 0;
        _mixedLoraNextMs = millis() + rngRange(15000, 30000);
        mixConfigElrs();
        _mixLastPktUs = micros();
        // Transmit first ELRS packet
        mixElrsTx();
        Serial.println("FP: Mixed mode started (ELRS 40ch + LoRaWAN)");
        break;

    default:
        return;
    }

    _fpRunning = true;
}

void fpStop() {
    if (!_radio) return;
    _radio->standby();
    _fpRunning = false;

    // Reconfigure radio back to default for other modes
    _radio->begin(915.0, 125.0, 9, 7,
                  SYNC_WORD_ELRS, 10, 8, 1.8, false);

    Serial.printf("FP stopped: %lu LoRaWAN, %lu bursts, %lu ELRS\n",
                  (unsigned long)_loraCount,
                  (unsigned long)_burstCount,
                  (unsigned long)_mixedElrsCount);
}

void fpUpdate() {
    if (!_fpRunning || !_radio) return;

    unsigned long nowMs = millis();

    switch (_fpMode) {
    case FP_LORAWAN:
        if (nowMs >= _loraNextTxMs) {
            lorawanTransmit();
        }
        break;

    case FP_ISM_BURST:
        if (nowMs >= _burstNextMs) {
            ismBurstTransmit();
        }
        break;

    case FP_MIXED: {
        // LoRaWAN burst interrupt (every 30-60s)
        if (nowMs >= _mixedLoraNextMs) {
            mixLorawanBurst();
            _mixLastPktUs = micros();
            _mixPktsSinceHop = 0;  // reset after radio reconfig
        }

        // ELRS packets at configured rate — all from protocol_params.h
        uint32_t mixPktUs = 1000000UL / _mixRate.rateHz;
        unsigned long nowUs = micros();
        if ((nowUs - _mixLastPktUs) >= mixPktUs) {
            _mixLastPktUs = nowUs;
            mixElrsTx();
        }
        break;
    }

    default:
        break;
    }
}

FpParams fpGetParams() {
    return FpParams{
        _fpMode,
        _loraCount,
        _burstCount,
        _mixedElrsCount,
        _lastFreq,
        _lastSF,
        rfGetPower(),
        _fpRunning,
    };
}
