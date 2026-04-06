#include <Arduino.h>
#include "rf_modes.h"
#include "protocol_params.h"

// ============================================================
// CW Tone Generator — continuous carrier via SX1262
// ============================================================

static SX1262 *_radio = nullptr;
static bool _txOn = false;
static uint8_t _freqIndex = 0;

// ----- Frequency presets (MHz) -----
// These are the test frequencies SENTRY-RF's spectrum scanner
// should detect. Cycle through them with short-press during TX.
//
// TODO(user): Adjust this table to match your SENTRY-RF scan bands.
//   - 915.0: US ISM band center (ELRS/Crossfire home)
//   - 868.0: EU ISM band center
//   - 903.0: ELRS band start (US)
//   - 925.0: ELRS band end (US)
//   - 906.4: ELRS channel center (US band)
static const float CW_FREQ_PRESETS[] = {
    915.0f,
    868.0f,
    903.0f,
    925.0f,
    906.4f,
};
static const uint8_t CW_FREQ_COUNT = sizeof(CW_FREQ_PRESETS) / sizeof(CW_FREQ_PRESETS[0]);

// TX power in dBm — SX1262 range: -9 to +22
// Start at +10 dBm (~10 mW) for bench testing; plenty for 1-2 m range
static const int8_t CW_DEFAULT_POWER = 10;

static int8_t _powerDbm = CW_DEFAULT_POWER;

// ============================================================
// Implementation
// ============================================================

void cwInit(SX1262 *radio) {
    _radio = radio;
    _txOn = false;
    _freqIndex = 0;
    _powerDbm = CW_DEFAULT_POWER;
}

void cwStart() {
    if (!_radio) return;

    // Full reset + begin ensures clean state from any prior mode
    _radio->standby();
    _radio->reset();
    delay(100);
    _radio->begin(CW_FREQ_PRESETS[_freqIndex], 125.0, 9, 7,
                  RADIOLIB_SX126X_SYNC_WORD_PRIVATE, _powerDbm, 8, 1.8, false);

    _radio->setOutputPower(_powerDbm);

    // transmitDirect(0) = unmodulated carrier at the configured frequency
    int state = _radio->transmitDirect(0);
    if (state == RADIOLIB_ERR_NONE) {
        _txOn = true;
        Serial.printf("CW TX ON: %.2f MHz @ %d dBm\n", CW_FREQ_PRESETS[_freqIndex], _powerDbm);
    } else {
        Serial.printf("CW TX FAILED (error %d)\n", state);
        _txOn = false;
    }
}

void cwStop() {
    if (!_radio) return;

    _radio->standby();
    _txOn = false;
    Serial.println("CW TX OFF");
}

void cwCycleFreq() {
    _freqIndex = (_freqIndex + 1) % CW_FREQ_COUNT;

    // If currently transmitting, retune live
    if (_txOn) {
        float freq = CW_FREQ_PRESETS[_freqIndex];
        _radio->standby();
        _radio->setFrequency(freq);
        _radio->transmitDirect(0);
        Serial.printf("CW retune: %.2f MHz\n", freq);
    }
}

CwParams cwGetParams() {
    return CwParams{
        CW_FREQ_PRESETS[_freqIndex],
        _powerDbm,
        _txOn,
    };
}

// ============================================================
// Frequency Sweep — linear scan across 860–930 MHz
// ============================================================

static constexpr float SWEEP_START_MHZ = 860.0f;
static constexpr float SWEEP_END_MHZ   = 930.0f;

// Step size presets (MHz) — user cycles through with button
static const float SWEEP_STEP_PRESETS[] = { 0.1f, 0.25f, 0.5f, 1.0f, 5.0f };
static const uint8_t SWEEP_STEP_COUNT = sizeof(SWEEP_STEP_PRESETS) / sizeof(SWEEP_STEP_PRESETS[0]);

// Dwell time presets (microseconds) — time on each frequency
static const uint16_t SWEEP_DWELL_PRESETS[] = { 500, 1000, 5000, 10000, 50000 };
static const uint8_t SWEEP_DWELL_COUNT = sizeof(SWEEP_DWELL_PRESETS) / sizeof(SWEEP_DWELL_PRESETS[0]);

static bool     _sweepRunning  = false;
static uint8_t  _stepPreset    = 0;   // index into SWEEP_STEP_PRESETS
static uint8_t  _dwellPreset   = 0;   // index into SWEEP_DWELL_PRESETS
static float    _sweepCurrent  = SWEEP_START_MHZ;
static uint16_t _sweepStepIdx  = 0;
static unsigned long _lastStepUs = 0;
static uint32_t _sweepCount    = 0;   // completed full sweeps

static uint16_t sweepTotalSteps() {
    float step = SWEEP_STEP_PRESETS[_stepPreset];
    return (uint16_t)((SWEEP_END_MHZ - SWEEP_START_MHZ) / step);
}

void sweepStart() {
    if (!_radio) return;

    // Full reset ensures clean state when switching from LoRa/FSK modes
    _radio->standby();
    _radio->reset();
    delay(100);
    _radio->begin(SWEEP_START_MHZ, 125.0, 9, 7,
                  RADIOLIB_SX126X_SYNC_WORD_PRIVATE, _powerDbm, 8, 1.8, false);

    _sweepCurrent = SWEEP_START_MHZ;
    _sweepStepIdx = 0;
    _sweepCount = 0;

    _radio->setOutputPower(_powerDbm);
    _radio->setFrequency(_sweepCurrent);
    int state = _radio->transmitDirect(0);

    if (state == RADIOLIB_ERR_NONE) {
        _sweepRunning = true;
        _lastStepUs = micros();
        Serial.printf("SWEEP ON: %.1f→%.1f MHz, step=%.2f MHz, dwell=%u us\n",
                      SWEEP_START_MHZ, SWEEP_END_MHZ,
                      SWEEP_STEP_PRESETS[_stepPreset],
                      SWEEP_DWELL_PRESETS[_dwellPreset]);
    } else {
        Serial.printf("SWEEP TX FAILED (error %d)\n", state);
        _sweepRunning = false;
    }
}

void sweepStop() {
    if (!_radio) return;

    _radio->standby();
    _sweepRunning = false;
    Serial.printf("SWEEP OFF after %lu full sweeps\n", (unsigned long)_sweepCount);
}

void sweepUpdate() {
    if (!_sweepRunning || !_radio) return;

    unsigned long nowUs = micros();
    uint16_t dwell = SWEEP_DWELL_PRESETS[_dwellPreset];

    // micros() wraps at ~70 min — subtraction handles rollover correctly
    if ((nowUs - _lastStepUs) < dwell) return;

    _lastStepUs = nowUs;

    // Advance to next frequency step
    float step = SWEEP_STEP_PRESETS[_stepPreset];
    _sweepCurrent += step;
    _sweepStepIdx++;

    // Wrap back to start when we reach the end
    if (_sweepCurrent > SWEEP_END_MHZ) {
        _sweepCurrent = SWEEP_START_MHZ;
        _sweepStepIdx = 0;
        _sweepCount++;
    }

    // Retune: standby → setFreq → transmitDirect is the safe sequence
    // for the SX1262 PLL to settle cleanly
    _radio->standby();
    _radio->setFrequency(_sweepCurrent);
    _radio->transmitDirect(0);
}

void sweepCycleStep() {
    _stepPreset = (_stepPreset + 1) % SWEEP_STEP_COUNT;
    // Reset to start if sweep is running so step count stays coherent
    if (_sweepRunning) {
        _sweepCurrent = SWEEP_START_MHZ;
        _sweepStepIdx = 0;
    }
    Serial.printf("Sweep step: %.2f MHz\n", SWEEP_STEP_PRESETS[_stepPreset]);
}

void sweepCycleDwell() {
    _dwellPreset = (_dwellPreset + 1) % SWEEP_DWELL_COUNT;
    Serial.printf("Sweep dwell: %u us\n", SWEEP_DWELL_PRESETS[_dwellPreset]);
}

SweepParams sweepGetParams() {
    return SweepParams{
        SWEEP_START_MHZ,
        SWEEP_END_MHZ,
        SWEEP_STEP_PRESETS[_stepPreset],
        _sweepCurrent,
        SWEEP_DWELL_PRESETS[_dwellPreset],
        _sweepStepIdx,
        sweepTotalSteps(),
        _powerDbm,
        _sweepRunning,
    };
}

// ============================================================
// Shared Power Control — used by all TX modes
// ============================================================

static const int8_t POWER_PRESETS[] = { -9, 0, 5, 10, 14, 17, 20, 22 };
static const uint8_t POWER_COUNT = sizeof(POWER_PRESETS) / sizeof(POWER_PRESETS[0]);
static uint8_t _powerPreset = 3;  // index 3 = +10 dBm default

void rfCyclePower() {
    _powerPreset = (_powerPreset + 1) % POWER_COUNT;
    _powerDbm = POWER_PRESETS[_powerPreset];
    Serial.printf("TX power: %d dBm\n", _powerDbm);
}

int8_t rfGetPower() {
    return _powerDbm;
}

// ============================================================
// ELRS 915 MHz FHSS Simulation
// ============================================================
// All protocol constants from protocol_params.h (single source of truth).
// Domain and air rate are selected at runtime via pointers.

static const ElrsDomain*  _elrsDomain  = &ELRS_DOMAINS[ELRS_DOMAIN_FCC915];
static const ElrsAirRate* _elrsRate    = &ELRS_AIR_RATES[ELRS_RATE_200HZ];

// FHSS sequence — pseudo-random permutation generated via LCG Fisher-Yates
static uint8_t _elrsHopSeq[80];  // sized for largest domain (ISM2G4=80)
static uint8_t _elrsHopIdx = 0;

static bool     _elrsRunning    = false;
static uint32_t _elrsPacketCount = 0;
static uint32_t _elrsHopCount   = 0;
static uint8_t  _elrsPktsSinceHop = 0;
static float    _elrsCurrentMHz = 0;
static unsigned long _elrsLastPktUs = 0;

// Dummy payloads matching real ELRS packet sizes per air rate — v2 §3.1.2
static const uint8_t ELRS_PAYLOAD_8[]  = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };
static const uint8_t ELRS_PAYLOAD_10[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA, 0x55, 0xBB };

// Binding/beacon state machine — v2 §3.1.7
enum ElrsState { ELRS_STATE_CONNECTED, ELRS_STATE_BINDING };
static ElrsState _elrsState = ELRS_STATE_CONNECTED;
static unsigned long _bindingStartMs = 0;
static const uint32_t BINDING_DURATION_MS = 10000;  // 10 seconds on sync channel
static unsigned long _bindingLastTxMs = 0;

void elrsSetRate(uint8_t rateIndex) {
    if (rateIndex < ELRS_AIR_RATE_COUNT) {
        _elrsRate = &ELRS_AIR_RATES[rateIndex];
    }
}

void elrsSetDomain(uint8_t domIndex) {
    if (domIndex < ELRS_DOMAIN_COUNT && domIndex != ELRS_DOMAIN_ISM2G4) {
        _elrsDomain = &ELRS_DOMAINS[domIndex];
    }
}

// Effective hop interval: DVDA always uses 2, standard uses domain's value
static uint8_t elrsEffectiveHopInterval() {
    return _elrsRate->isDvda ? _elrsRate->hopInterval : _elrsDomain->hopInterval;
}

// Build pseudo-random hop sequence using LCG-seeded Fisher-Yates shuffle.
// Uses real ELRS LCG constants (0x343FD / 0x269EC3) from protocol_params.h.
// Sync channel inserted at position 0, visited every freq_count hops.
static void elrsBuildHopSequence(uint32_t seed) {
    uint8_t numCh = _elrsDomain->channels;
    for (uint8_t i = 0; i < numCh; i++) {
        _elrsHopSeq[i] = i;
    }

    // Fisher-Yates shuffle with real ELRS LCG constants — v2 §3.1.6
    uint32_t rng = seed;
    for (uint8_t i = numCh - 1; i > 0; i--) {
        rng = rng * ELRS_LCG_MULTIPLIER + ELRS_LCG_INCREMENT;
        uint8_t j = (rng >> 16) % (i + 1);  // use upper bits for better distribution
        uint8_t tmp = _elrsHopSeq[i];
        _elrsHopSeq[i] = _elrsHopSeq[j];
        _elrsHopSeq[j] = tmp;
    }

    // Sync channel at position 0 — v2 §3.1.6
    _elrsHopSeq[0] = _elrsDomain->syncChannel;
}

void elrsStart() {
    if (!_radio) return;

    const ElrsDomain&  dom  = *_elrsDomain;
    const ElrsAirRate& rate = *_elrsRate;

    // Build hop sequence with a fixed seed (simulates one binding phrase)
    elrsBuildHopSequence(0xDEADBEEF);
    _elrsHopIdx = 0;
    _elrsPacketCount = 0;
    _elrsHopCount = 0;
    _elrsPktsSinceHop = 0;

    // Full reset ensures clean state when switching from CW/sweep modes
    _radio->reset();
    delay(100);

    // Configure radio from protocol_params.h structs
    float startFreq = elrsChanFreq(dom, _elrsHopSeq[0]);
    int state = _radio->begin(
        startFreq,
        (float)(rate.bwHz / 1000),   // BW in kHz
        rate.sf,
        rate.cr,                      // CR 4/7 per v2 §3.1.2
        SYNC_WORD_ELRS,               // 0x12 per v2 §6.3
        _powerDbm,
        rate.preambleLen,             // 6 symbols per v2 §3.1.2
        1.8,                          // TCXO voltage — T3-S3 has 1.8V TCXO
        false                         // use LDO (not DC-DC)
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ELRS] radio config FAILED (error %d)\n", state);
        _elrsRunning = false;
        return;
    }

    _radio->explicitHeader();
    _radio->setCurrentLimit(140.0);

    _elrsCurrentMHz = startFreq;
    _elrsRunning = true;
    _elrsLastPktUs = micros();

    // Select payload size based on air rate — v2 §3.1.2
    const uint8_t* payload = (rate.payloadLen <= 8) ? ELRS_PAYLOAD_8 : ELRS_PAYLOAD_10;
    _radio->startTransmit(payload, rate.payloadLen);
    _elrsPacketCount++;
    _elrsPktsSinceHop = 1;

    _elrsState = ELRS_STATE_CONNECTED;

    // Protocol info output per v2 §7.2
    uint8_t effHop = elrsEffectiveHopInterval();
    uint32_t pktIntervalUs = 1000000UL / rate.rateHz;
    uint32_t dwellMs = (effHop * pktIntervalUs) / 1000;
    uint32_t hopsPerSec = rate.rateHz / effHop;
    Serial.printf("[ELRS-%s] %uch %.1f-%.1fMHz SF%u/BW%lu %uHz hop_every_%u sync_ch=%u\n",
                  dom.name, dom.channels, dom.freqStartMHz, dom.freqStopMHz,
                  rate.sf, rate.bwHz / 1000, rate.rateHz, effHop, dom.syncChannel);
    Serial.printf("  Dwell: %lums/freq  Hops: %lu/s  Preamble: %usym  SyncWord: 0x%02X  Power: %d dBm\n",
                  (unsigned long)dwellMs, (unsigned long)hopsPerSec,
                  rate.preambleLen, SYNC_WORD_ELRS, _powerDbm);

    // EU868 duty cycle warning — v2 §2.2
    if (_elrsDomain == &ELRS_DOMAINS[ELRS_DOMAIN_EU868]) {
        Serial.println("  WARNING: EU868 ETSI duty cycle limits (1%) apply in real deployments");
    }
}

// Start in binding/beacon state — v2 §3.1.7
// Transmits on sync channel at 1 Hz for 10 seconds, then transitions to FHSS
void elrsStartBinding() {
    if (!_radio) return;

    const ElrsDomain&  dom  = *_elrsDomain;
    const ElrsAirRate& rate = *_elrsRate;

    _radio->reset();
    delay(100);

    // Configure radio on sync channel frequency
    float syncFreq = elrsChanFreq(dom, dom.syncChannel);
    int state = _radio->begin(
        syncFreq,
        (float)(rate.bwHz / 1000), rate.sf, rate.cr,
        SYNC_WORD_ELRS, _powerDbm, rate.preambleLen, 1.8, false
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ELRS] radio config FAILED (error %d)\n", state);
        _elrsRunning = false;
        return;
    }

    _radio->explicitHeader();
    _radio->setCurrentLimit(140.0);

    _elrsCurrentMHz = syncFreq;
    _elrsRunning = true;
    _elrsState = ELRS_STATE_BINDING;
    _bindingStartMs = millis();
    _bindingLastTxMs = 0;
    _elrsPacketCount = 0;
    _elrsHopCount = 0;
    _elrsPktsSinceHop = 0;

    Serial.printf("[ELRS-%s] BINDING: TX on sync channel (%.1f MHz) 1 Hz...\n",
                  dom.name, syncFreq);
}

void elrsStop() {
    if (!_radio) return;

    _radio->standby();
    _elrsRunning = false;
    Serial.printf("[ELRS] TX OFF: %lu packets, %lu hops (%.1f pkts/hop avg)\n",
                  (unsigned long)_elrsPacketCount,
                  (unsigned long)_elrsHopCount,
                  _elrsHopCount > 0 ? (float)_elrsPacketCount / _elrsHopCount : 0);
}

void elrsUpdate() {
    if (!_elrsRunning || !_radio) return;

    // --- Binding state: TX on sync channel at 1 Hz, then transition to FHSS ---
    if (_elrsState == ELRS_STATE_BINDING) {
        unsigned long nowMs = millis();

        // Check for transition to connected
        if ((nowMs - _bindingStartMs) >= BINDING_DURATION_MS) {
            Serial.printf("[ELRS-%s] BINDING: 10s elapsed, transitioning to FHSS...\n",
                          _elrsDomain->name);

            // Build hop sequence and switch to connected FHSS
            elrsBuildHopSequence(0xDEADBEEF);
            _elrsHopIdx = 0;
            _elrsHopCount = 0;
            _elrsPktsSinceHop = 0;
            _elrsLastPktUs = micros();
            _elrsState = ELRS_STATE_CONNECTED;

            Serial.printf("[ELRS-%s] CONNECTED: %uch FHSS active\n", _elrsDomain->name,
                          _elrsDomain->channels);
            return;
        }

        // Transmit beacon at 1 Hz on sync channel
        if ((nowMs - _bindingLastTxMs) >= 1000) {
            _bindingLastTxMs = nowMs;
            _radio->startTransmit(ELRS_PAYLOAD_8, 8);
            _elrsPacketCount++;
        }
        return;
    }

    // --- Connected state: normal FHSS hopping ---
    uint32_t pktIntervalUs = 1000000UL / _elrsRate->rateHz;
    unsigned long nowUs = micros();
    if ((nowUs - _elrsLastPktUs) < pktIntervalUs) return;

    _elrsLastPktUs += pktIntervalUs;  // accumulate to prevent drift

    uint8_t numCh    = _elrsDomain->channels;
    uint8_t hopEvery = elrsEffectiveHopInterval();

    // Hop every N packets (N from domain for standard, 2 for DVDA)
    if (_elrsPktsSinceHop >= hopEvery) {
        _elrsPktsSinceHop = 0;
        _elrsHopIdx = (_elrsHopIdx + 1) % numCh;
        _elrsHopCount++;

        // Sync channel: every freq_count hops, force to sync channel
        uint8_t nextChan;
        if ((_elrsHopCount % numCh) == 0) {
            nextChan = _elrsDomain->syncChannel;
        } else {
            nextChan = _elrsHopSeq[_elrsHopIdx];
        }

        float nextFreq = elrsChanFreq(*_elrsDomain, nextChan);
        _elrsCurrentMHz = nextFreq;

        _radio->standby();
        _radio->setFrequency(nextFreq);
    }

    const uint8_t* payload = (_elrsRate->payloadLen <= 8) ? ELRS_PAYLOAD_8 : ELRS_PAYLOAD_10;
    _radio->startTransmit(payload, _elrsRate->payloadLen);
    _elrsPacketCount++;
    _elrsPktsSinceHop++;
}

ElrsParams elrsGetParams() {
    return ElrsParams{
        _elrsCurrentMHz,
        _elrsHopSeq[_elrsHopIdx],
        _elrsPacketCount,
        _elrsHopCount,
        _powerDbm,
        _elrsRunning,
    };
}
