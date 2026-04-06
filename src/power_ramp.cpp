#include <Arduino.h>
#include "power_ramp.h"

// ============================================================
// Power Ramp — drone approach simulation via ELRS FHSS
// ============================================================

static SX1262 *_radio = nullptr;

// ELRS FCC915 parameters (matches rf_modes.cpp)
static constexpr uint8_t  ELRS_NUM_CHANNELS     = 40;
static constexpr float    ELRS_BAND_START       = 903.5f;
static constexpr float    ELRS_BAND_END         = 926.9f;
static constexpr float    ELRS_CHAN_SPACING      = (ELRS_BAND_END - ELRS_BAND_START) / ELRS_NUM_CHANNELS;
static constexpr uint32_t ELRS_PACKET_INTERVAL_US = 5000;
static constexpr uint8_t  ELRS_HOP_EVERY_N      = 4;
static constexpr uint8_t  ELRS_SYNC_CHANNEL     = 20;

static uint8_t _hopSeq[ELRS_NUM_CHANNELS];
static uint8_t _hopIdx = 0;

// Power ramp state
static constexpr int8_t PWR_MIN = -9;
static constexpr int8_t PWR_MAX = 22;
static constexpr uint32_t HOLD_AT_MAX_SEC = 10;

static bool     _running       = false;
static uint32_t _packetCount   = 0;
static uint32_t _hopCount      = 0;
static uint8_t  _pktsSinceHop  = 0;
static float    _currentMHz    = ELRS_BAND_START;
static int8_t   _currentPwr    = PWR_MIN;
static bool     _ascending     = true;
static unsigned long _startMs  = 0;
static unsigned long _lastHopUs = 0;
static unsigned long _lastPwrMs = 0;

// Duration presets
static const uint32_t DURATION_PRESETS[] = { 30, 60, 120, 300 };
static const uint8_t DURATION_COUNT = sizeof(DURATION_PRESETS) / sizeof(DURATION_PRESETS[0]);
static uint8_t _durationIdx = 1;  // default 60s
static uint32_t _rampDuration = 60;

// Dummy ELRS payload
static const uint8_t RAMP_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00 };

static void buildHopSequence() {
    for (uint8_t i = 0; i < ELRS_NUM_CHANNELS; i++) _hopSeq[i] = i;
    uint32_t rng = 0xCAFEBABE;
    for (uint8_t i = ELRS_NUM_CHANNELS - 1; i > 0; i--) {
        rng = rng * 1664525UL + 1013904223UL;
        uint8_t j = rng % (i + 1);
        uint8_t tmp = _hopSeq[i];
        _hopSeq[i] = _hopSeq[j];
        _hopSeq[j] = tmp;
    }
    _hopSeq[0] = ELRS_SYNC_CHANNEL;
}

static float chanToFreq(uint8_t chan) {
    return ELRS_BAND_START + (chan * ELRS_CHAN_SPACING);
}

void powerRampInit(SX1262 *radio) {
    _radio = radio;
    _running = false;
}

void powerRampStart() {
    if (!_radio) return;

    buildHopSequence();
    _hopIdx = 0;
    _packetCount = 0;
    _hopCount = 0;
    _pktsSinceHop = 0;
    _currentPwr = PWR_MIN;
    _ascending = true;

    // Full reset + configure for ELRS LoRa SF6 BW500
    _radio->reset();
    delay(100);

    int state = _radio->begin(
        chanToFreq(_hopSeq[0]),
        500.0, 6, 5,
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
        PWR_MIN, 8, 1.8, false
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("RAMP: radio config FAILED (error %d)\n", state);
        _running = false;
        return;
    }

    _radio->explicitHeader();
    _radio->setCurrentLimit(140.0);

    _currentMHz = chanToFreq(_hopSeq[0]);
    _startMs = millis();
    _lastHopUs = micros();
    _lastPwrMs = millis();
    _running = true;

    _radio->transmit(RAMP_PAYLOAD, sizeof(RAMP_PAYLOAD));
    _packetCount++;

    Serial.printf("RAMP: Started, %d→%d dBm over %lu sec, ELRS FHSS\n",
                  PWR_MIN, PWR_MAX, (unsigned long)_rampDuration);
}

void powerRampStop() {
    if (!_radio) return;
    _radio->standby();
    _running = false;
    Serial.printf("RAMP: Stopped at %d dBm, %lu packets\n",
                  _currentPwr, (unsigned long)_packetCount);
}

void powerRampUpdate() {
    if (!_running || !_radio) return;

    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    // FHSS packets at 200 Hz, hop every 4 packets
    if ((nowUs - _lastHopUs) >= ELRS_PACKET_INTERVAL_US) {
        _lastHopUs = nowUs;

        // Hop every 4 packets
        if (_pktsSinceHop >= ELRS_HOP_EVERY_N) {
            _pktsSinceHop = 0;
            _hopIdx = (_hopIdx + 1) % ELRS_NUM_CHANNELS;
            _hopCount++;

            // Sync channel every freq_count hops
            uint8_t nextChan;
            if ((_hopCount % ELRS_NUM_CHANNELS) == 0) {
                nextChan = ELRS_SYNC_CHANNEL;
            } else {
                nextChan = _hopSeq[_hopIdx];
            }

            _currentMHz = chanToFreq(nextChan);
            _radio->setFrequency(_currentMHz);
        }

        _radio->transmit(RAMP_PAYLOAD, sizeof(RAMP_PAYLOAD));
        _packetCount++;
        _pktsSinceHop++;
    }

    // Update power every 500ms
    if ((nowMs - _lastPwrMs) >= 500) {
        _lastPwrMs = nowMs;

        uint32_t elapsedSec = (nowMs - _startMs) / 1000;
        uint32_t totalCycle = _rampDuration + HOLD_AT_MAX_SEC + _rampDuration;

        if (elapsedSec >= totalCycle) {
            // Full cycle complete — restart
            _startMs = nowMs;
            _currentPwr = PWR_MIN;
            _ascending = true;
            Serial.println("RAMP: Cycle complete, restarting");
        } else if (elapsedSec < _rampDuration) {
            // Ascending: -9 → +22
            float frac = (float)elapsedSec / (float)_rampDuration;
            _currentPwr = PWR_MIN + (int8_t)(frac * (PWR_MAX - PWR_MIN));
            _ascending = true;
        } else if (elapsedSec < _rampDuration + HOLD_AT_MAX_SEC) {
            // Hold at max
            _currentPwr = PWR_MAX;
            _ascending = true;
        } else {
            // Descending: +22 → -9
            uint32_t descElapsed = elapsedSec - _rampDuration - HOLD_AT_MAX_SEC;
            float frac = (float)descElapsed / (float)_rampDuration;
            _currentPwr = PWR_MAX - (int8_t)(frac * (PWR_MAX - PWR_MIN));
            _ascending = false;
        }

        _radio->setOutputPower(_currentPwr);
    }
}

void powerRampCycleDuration() {
    _durationIdx = (_durationIdx + 1) % DURATION_COUNT;
    _rampDuration = DURATION_PRESETS[_durationIdx];
    Serial.printf("RAMP: duration set to %lu sec\n", (unsigned long)_rampDuration);
}

PowerRampParams powerRampGetParams() {
    return PowerRampParams{
        _currentMHz,
        _currentPwr,
        _packetCount,
        _running ? (uint32_t)((millis() - _startMs) / 1000) : 0,
        _rampDuration,
        _ascending,
        _running,
    };
}
