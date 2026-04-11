#include <Arduino.h>
#include "power_ramp.h"
#include "protocol_params.h"

// ============================================================
// Power Ramp — drone approach simulation via ELRS FHSS
// ============================================================

static SX1262 *_radio = nullptr;

// ELRS parameters from protocol_params.h — no local duplicates
static const ElrsDomain&  _rampDom  = ELRS_DOMAINS[ELRS_DOMAIN_FCC915];
static const ElrsAirRate& _rampRate = ELRS_AIR_RATES[ELRS_RATE_200HZ];

// Sequence holds non-sync channels only; sync is interleaved at runtime.
static uint8_t _hopSeq[80];  // sized for largest domain
static uint8_t _seqLen = 0;  // = numCh - 1 after build
static uint8_t _hopIdx = 0;

// Power ramp state
static constexpr int8_t PWR_MIN = -9;
static constexpr int8_t PWR_MAX = 22;
static constexpr uint32_t HOLD_AT_MAX_SEC = 10;

static bool     _running       = false;
static uint32_t _packetCount   = 0;
static uint32_t _hopCount      = 0;
static uint8_t  _pktsSinceHop  = 0;
static float    _currentMHz    = 0;
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

// 8-byte payload matching real ELRS 200 Hz packet size — v2 §3.1.2
static const uint8_t RAMP_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };

static void buildHopSequence() {
    uint8_t numCh  = _rampDom.channels;
    uint8_t syncCh = _rampDom.syncChannel;

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < numCh; i++) {
        if (i != syncCh) _hopSeq[cnt++] = i;
    }
    _seqLen = cnt;

    uint32_t rng = 0xCAFEBABE;
    for (uint8_t i = cnt - 1; i > 0; i--) {
        rng = rng * ELRS_LCG_MULTIPLIER + ELRS_LCG_INCREMENT;
        uint8_t j = (rng >> 16) % (i + 1);
        uint8_t tmp = _hopSeq[i];
        _hopSeq[i] = _hopSeq[j];
        _hopSeq[j] = tmp;
    }
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

    // Full reset + configure from protocol_params.h
    _radio->reset();
    delay(100);

    int state = _radio->begin(
        elrsChanFreq(_rampDom, _hopSeq[0]),
        (float)(_rampRate.bwHz / 1000), _rampRate.sf, _rampRate.cr,
        SYNC_WORD_ELRS, PWR_MIN, _rampRate.preambleLen, 1.8, false
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("RAMP: radio config FAILED (error %d)\n", state);
        _running = false;
        return;
    }

    // Real ELRS uses implicit header (v2 §3.1.4). RAMP_PAYLOAD is an
    // 8-byte packet and _rampRate is fixed at 200 Hz (payloadLen=8).
    _radio->implicitHeader(_rampRate.payloadLen);
    _radio->setCurrentLimit(140.0);

    _currentMHz = elrsChanFreq(_rampDom, _hopSeq[0]);
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

    // FHSS packets, hop every N packets — all from protocol_params.h
    uint32_t pktIntervalUs = 1000000UL / _rampRate.rateHz;
    if ((nowUs - _lastHopUs) >= pktIntervalUs) {
        _lastHopUs = nowUs;

        uint8_t numCh    = _rampDom.channels;
        uint8_t hopEvery = _rampRate.hopInterval;

        if (_pktsSinceHop >= hopEvery) {
            _pktsSinceHop = 0;
            _hopCount++;

            uint8_t nextChan;
            if ((_hopCount % numCh) == 0) {
                nextChan = _rampDom.syncChannel;
            } else {
                _hopIdx = (uint8_t)((_hopIdx + 1) % _seqLen);
                nextChan = _hopSeq[_hopIdx];
            }
            _currentMHz = elrsChanFreq(_rampDom, nextChan);
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
