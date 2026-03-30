#include <Arduino.h>
#include "rf_modes.h"

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
// Real ELRS 200Hz mode: LoRa SF6 BW500, 80 channels across
// 902–928 MHz, pseudo-random hop sequence, 5 ms per channel.

static constexpr uint8_t  ELRS_NUM_CHANNELS = 80;
static constexpr float    ELRS_BAND_START   = 902.0f;  // MHz
static constexpr float    ELRS_BAND_END     = 928.0f;  // MHz
static constexpr float    ELRS_CHAN_SPACING  = (ELRS_BAND_END - ELRS_BAND_START) / ELRS_NUM_CHANNELS;
static constexpr uint32_t ELRS_HOP_INTERVAL_US = 5000;  // 5 ms = 200 Hz hop rate

// FHSS sequence — pseudo-random permutation of 0..79
// Generated once at start via seeded Fisher-Yates shuffle
static uint8_t _elrsHopSeq[ELRS_NUM_CHANNELS];
static uint8_t _elrsHopIdx = 0;

static bool     _elrsRunning    = false;
static uint32_t _elrsPacketCount = 0;
static uint32_t _elrsHopCount   = 0;
static float    _elrsCurrentMHz = ELRS_BAND_START;
static unsigned long _elrsLastHopUs = 0;

// 8-byte dummy payload matching real ELRS packet size
static const uint8_t ELRS_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };

// Build a pseudo-random hop sequence using Fisher-Yates shuffle.
// Seed can be changed to simulate different ELRS binding UIDs.
static void elrsBuildHopSequence(uint32_t seed) {
    // Start with sequential channel indices
    for (uint8_t i = 0; i < ELRS_NUM_CHANNELS; i++) {
        _elrsHopSeq[i] = i;
    }

    // Fisher-Yates shuffle with simple LCG PRNG
    uint32_t rng = seed;
    for (uint8_t i = ELRS_NUM_CHANNELS - 1; i > 0; i--) {
        rng = rng * 1664525UL + 1013904223UL;  // Knuth LCG
        uint8_t j = rng % (i + 1);
        uint8_t tmp = _elrsHopSeq[i];
        _elrsHopSeq[i] = _elrsHopSeq[j];
        _elrsHopSeq[j] = tmp;
    }
}

static float elrsChanToFreq(uint8_t chan) {
    return ELRS_BAND_START + (chan * ELRS_CHAN_SPACING);
}

void elrsStart() {
    if (!_radio) return;

    // Build hop sequence with a fixed seed (simulates one binding phrase)
    elrsBuildHopSequence(0xDEADBEEF);
    _elrsHopIdx = 0;
    _elrsPacketCount = 0;
    _elrsHopCount = 0;

    // Full reset ensures clean state when switching from CW/sweep modes
    _radio->reset();
    delay(100);

    // Configure radio for ELRS 200Hz mode: LoRa SF6 BW500
    // TCXO voltage 1.8V is required for LilyGo T3-S3 after hardware reset
    int state = _radio->begin(
        elrsChanToFreq(_elrsHopSeq[0]),  // start on first hop channel
        500.0,    // BW 500 kHz
        6,        // SF6
        5,        // CR 4/5
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
        _powerDbm,
        8,        // preamble length
        1.8,      // TCXO voltage — T3-S3 has 1.8V TCXO
        false     // use LDO (not DC-DC)
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("ELRS radio config FAILED (error %d)\n", state);
        _elrsRunning = false;
        return;
    }

    // SF6 requires explicit header mode and fixed packet length
    _radio->explicitHeader();
    _radio->setCurrentLimit(140.0);

    _elrsCurrentMHz = elrsChanToFreq(_elrsHopSeq[0]);
    _elrsRunning = true;
    _elrsLastHopUs = micros();

    // Transmit first packet immediately
    _radio->transmit(ELRS_PAYLOAD, sizeof(ELRS_PAYLOAD));
    _elrsPacketCount++;

    Serial.printf("ELRS TX ON: SF6 BW500, 80ch FHSS, 200Hz, %d dBm\n", _powerDbm);
}

void elrsStop() {
    if (!_radio) return;

    _radio->standby();
    _elrsRunning = false;
    Serial.printf("ELRS TX OFF: %lu packets, %lu hops\n",
                  (unsigned long)_elrsPacketCount,
                  (unsigned long)_elrsHopCount);
}

void elrsUpdate() {
    if (!_elrsRunning || !_radio) return;

    unsigned long nowUs = micros();
    if ((nowUs - _elrsLastHopUs) < ELRS_HOP_INTERVAL_US) return;

    _elrsLastHopUs = nowUs;

    // Advance to next channel in the hop sequence
    _elrsHopIdx = (_elrsHopIdx + 1) % ELRS_NUM_CHANNELS;
    _elrsHopCount++;

    float nextFreq = elrsChanToFreq(_elrsHopSeq[_elrsHopIdx]);
    _elrsCurrentMHz = nextFreq;

    // Hop: retune and transmit
    _radio->setFrequency(nextFreq);
    _radio->transmit(ELRS_PAYLOAD, sizeof(ELRS_PAYLOAD));
    _elrsPacketCount++;
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
