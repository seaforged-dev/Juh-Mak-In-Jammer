#include <Arduino.h>
#include "false_positive.h"
#include "rf_modes.h"   // for rfGetPower(), elrs reuse

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

static constexpr float LORAWAN_FREQ = 903.9f;  // US LoRaWAN uplink ch 0
static unsigned long _loraNextTxMs = 0;

// Randomized dummy payload (20-50 bytes, mimics sensor data)
static uint8_t _loraBuf[50];

static void lorawanConfigRadio() {
    // Pick a random SF between 7-12 for each transmission
    // Real LoRaWAN devices use ADR which varies SF
    _lastSF = (uint8_t)rngRange(7, 12);

    _radio->standby();
    _radio->begin(
        LORAWAN_FREQ,
        125.0,       // BW 125 kHz (standard LoRaWAN)
        _lastSF,
        5,           // CR 4/5
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
        rfGetPower(),
        8,           // preamble
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

    _lastFreq = LORAWAN_FREQ;
    _radio->transmit(_loraBuf, len);
    _loraCount++;

    // Schedule next TX in 30-60 seconds
    _loraNextTxMs = millis() + rngRange(30000, 60000);

    Serial.printf("LoRaWAN TX: SF%u BW125 @ %.1f MHz, %u bytes\n",
                  _lastSF, LORAWAN_FREQ, len);
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

// ELRS channel table (same as rf_modes.cpp — shared constants)
static constexpr uint8_t  MIX_ELRS_CHANNELS = 80;
static constexpr float    MIX_ELRS_START     = 902.0f;
static constexpr float    MIX_ELRS_END       = 928.0f;
static constexpr float    MIX_ELRS_SPACING   = (MIX_ELRS_END - MIX_ELRS_START) / MIX_ELRS_CHANNELS;
static constexpr uint32_t MIX_HOP_US         = 5000;  // 200 Hz

static uint8_t  _mixHopSeq[MIX_ELRS_CHANNELS];
static uint8_t  _mixHopIdx = 0;
static unsigned long _mixLastHopUs = 0;
static const uint8_t MIX_ELRS_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };

static void mixBuildHopSequence() {
    for (uint8_t i = 0; i < MIX_ELRS_CHANNELS; i++) _mixHopSeq[i] = i;
    uint32_t rng = 0xCAFEBABE;
    for (uint8_t i = MIX_ELRS_CHANNELS - 1; i > 0; i--) {
        rng = rng * 1664525UL + 1013904223UL;
        uint8_t j = rng % (i + 1);
        uint8_t tmp = _mixHopSeq[i];
        _mixHopSeq[i] = _mixHopSeq[j];
        _mixHopSeq[j] = tmp;
    }
}

static void mixConfigElrs() {
    _radio->standby();
    float freq = MIX_ELRS_START + (_mixHopSeq[_mixHopIdx] * MIX_ELRS_SPACING);
    _radio->begin(freq, 500.0, 6, 5,
                  RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                  rfGetPower(), 8, 1.8, false);
    _radio->explicitHeader();
    _mixedElrsConfigured = true;
}

static void mixElrsHop() {
    _mixHopIdx = (_mixHopIdx + 1) % MIX_ELRS_CHANNELS;
    float freq = MIX_ELRS_START + (_mixHopSeq[_mixHopIdx] * MIX_ELRS_SPACING);
    _lastFreq = freq;
    _radio->setFrequency(freq);
    _radio->transmit(MIX_ELRS_PAYLOAD, sizeof(MIX_ELRS_PAYLOAD));
    _mixedElrsCount++;
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
        _mixedLoraNextMs = millis() + rngRange(15000, 30000);
        mixConfigElrs();
        _mixLastHopUs = micros();
        // Transmit first ELRS packet
        mixElrsHop();
        Serial.println("FP: Mixed mode started (ELRS + LoRaWAN)");
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
                  RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8, 1.8, false);

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
            _mixLastHopUs = micros();
        }

        // ELRS FHSS hops at 200 Hz
        unsigned long nowUs = micros();
        if ((nowUs - _mixLastHopUs) >= MIX_HOP_US) {
            _mixLastHopUs = nowUs;
            mixElrsHop();
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
