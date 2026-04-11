#include <Arduino.h>
#include "combined_mode.h"
#include "rid_spoofer.h"
#include "rf_modes.h"
#include "protocol_params.h"

// ============================================================
// Combined Mode — RID (Core 0) + ELRS (Core 1 FreeRTOS task)
// ============================================================
//
// Architecture:
//   Core 0: WiFi + BLE (RID spoofer) — runs in loop() via combinedUpdate()
//   Core 1: SX1262 LoRa FHSS (ELRS) — runs in a pinned FreeRTOS task
//
// These don't conflict because WiFi/BLE use the internal radio
// and ELRS uses the external SX1262 over SPI — different HW buses.

static SX1262 *_radio = nullptr;
static bool _combinedRunning = false;
static unsigned long _startTimeMs = 0;
static TaskHandle_t _elrsTaskHandle = nullptr;

// ELRS task state (runs on Core 1)
static volatile bool _elrsTaskRunning = false;
static volatile uint32_t _elrsTaskPkts = 0;
static volatile uint32_t _elrsTaskHops = 0;

// ELRS parameters from protocol_params.h — no local duplicates
static const ElrsDomain&  _cmbDom  = ELRS_DOMAINS[ELRS_DOMAIN_FCC915];
static const ElrsAirRate& _cmbRate = ELRS_AIR_RATES[ELRS_RATE_200HZ];

// Sequence holds non-sync channels only; sync is interleaved at runtime.
static uint8_t _cmbHopSeq[80];  // sized for largest domain
static uint8_t _cmbSeqLen = 0;  // = numCh - 1 after build
static const uint8_t CMB_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00, 0x05, 0x7A, 0x3C, 0xAA };

static void cmbBuildHopSequence() {
    uint8_t numCh  = _cmbDom.channels;
    uint8_t syncCh = _cmbDom.syncChannel;

    uint8_t cnt = 0;
    for (uint8_t i = 0; i < numCh; i++) {
        if (i != syncCh) _cmbHopSeq[cnt++] = i;
    }
    _cmbSeqLen = cnt;

    uint32_t rng = 0xFEEDFACE;
    for (uint8_t i = cnt - 1; i > 0; i--) {
        rng = rng * ELRS_LCG_MULTIPLIER + ELRS_LCG_INCREMENT;
        uint8_t j = (rng >> 16) % (i + 1);
        uint8_t tmp = _cmbHopSeq[i];
        _cmbHopSeq[i] = _cmbHopSeq[j];
        _cmbHopSeq[j] = tmp;
    }
}

// FreeRTOS task running ELRS FHSS on Core 1
static void elrsTask(void *param) {
    SX1262 *radio = (SX1262 *)param;
    uint8_t hopIdx = 0;
    uint8_t pktsSinceHop = 0;
    uint32_t hopCount = 0;

    uint8_t  numCh    = _cmbDom.channels;
    uint8_t  hopEvery = _cmbRate.hopInterval;
    uint32_t pktUs    = 1000000UL / _cmbRate.rateHz;

    // Configure radio from protocol_params.h
    radio->standby();
    float freq = elrsChanFreq(_cmbDom, _cmbHopSeq[0]);
    radio->begin(freq, (float)(_cmbRate.bwHz / 1000), _cmbRate.sf, _cmbRate.cr,
                 SYNC_WORD_ELRS, rfGetPower(), _cmbRate.preambleLen, 1.8, false);
    // Real ELRS uses implicit header (v2 §3.1.4). CMB_PAYLOAD is always
    // 8 bytes and _cmbRate is fixed at 200 Hz FCC915 (payloadLen=8), so
    // the fixed length matches every transmit() call in this task.
    radio->implicitHeader(_cmbRate.payloadLen);

    unsigned long lastPktUs = micros();

    while (_elrsTaskRunning) {
        unsigned long nowUs = micros();
        if ((nowUs - lastPktUs) >= pktUs) {
            lastPktUs = nowUs;

            if (pktsSinceHop >= hopEvery) {
                pktsSinceHop = 0;
                hopCount++;
                _elrsTaskHops++;

                uint8_t nextChan;
                if ((hopCount % numCh) == 0) {
                    nextChan = _cmbDom.syncChannel;
                } else {
                    hopIdx = (uint8_t)((hopIdx + 1) % _cmbSeqLen);
                    nextChan = _cmbHopSeq[hopIdx];
                }
                freq = elrsChanFreq(_cmbDom, nextChan);
                radio->setFrequency(freq);
            }

            radio->transmit(CMB_PAYLOAD, sizeof(CMB_PAYLOAD));
            _elrsTaskPkts++;
            pktsSinceHop++;
        }

        vTaskDelay(1);
    }

    radio->standby();
    radio->begin(915.0, 125.0, 9, 7, SYNC_WORD_ELRS, 10, 8, 1.8, false);
    vTaskDelete(NULL);
}

// ============================================================
// Public API
// ============================================================

void combinedInit(SX1262 *radio) {
    _radio = radio;
    _combinedRunning = false;
}

void combinedStart() {
    if (!_radio) return;

    // Initialize RID if not already done
    ridStart();

    // Build ELRS hop sequence
    cmbBuildHopSequence();
    _elrsTaskPkts = 0;
    _elrsTaskHops = 0;

    // Launch ELRS on Core 1 as a pinned FreeRTOS task
    _elrsTaskRunning = true;
    xTaskCreatePinnedToCore(
        elrsTask,           // task function
        "elrs_tx",          // name
        4096,               // stack size
        (void *)_radio,     // parameter
        1,                  // priority (above idle)
        &_elrsTaskHandle,   // handle
        1                   // Core 1
    );

    _startTimeMs = millis();
    _combinedRunning = true;
    Serial.println("COMBINED: Started — RID on Core 0, ELRS on Core 1");
}

void combinedStop() {
    // Stop ELRS task first
    _elrsTaskRunning = false;
    if (_elrsTaskHandle) {
        // Give the task time to exit cleanly
        vTaskDelay(pdMS_TO_TICKS(50));
        _elrsTaskHandle = nullptr;
    }

    // Stop RID
    ridStop();

    _combinedRunning = false;
    Serial.printf("COMBINED: Stopped — ELRS: %lu pkts, %lu hops\n",
                  (unsigned long)_elrsTaskPkts,
                  (unsigned long)_elrsTaskHops);
}

void combinedUpdate() {
    if (!_combinedRunning) return;

    // RID runs on Core 0 (this core) via its update function
    ridUpdate();
}

CombinedParams combinedGetParams() {
    RidParams rp = ridGetParams();
    return CombinedParams{
        rp.wifiPackets,
        rp.blePackets,
        (uint32_t)_elrsTaskPkts,
        (uint32_t)_elrsTaskHops,
        _combinedRunning ? (uint32_t)((millis() - _startTimeMs) / 1000) : 0,
        _combinedRunning,
    };
}
