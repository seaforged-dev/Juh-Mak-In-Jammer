#include <Arduino.h>
#include "combined_mode.h"
#include "rid_spoofer.h"
#include "rf_modes.h"

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

// ELRS channel table (same as rf_modes.cpp)
static constexpr uint8_t  CMB_ELRS_CHANNELS = 80;
static constexpr float    CMB_ELRS_START    = 902.0f;
static constexpr float    CMB_ELRS_END      = 928.0f;
static constexpr float    CMB_ELRS_SPACING  = (CMB_ELRS_END - CMB_ELRS_START) / CMB_ELRS_CHANNELS;
static constexpr uint32_t CMB_HOP_US        = 5000;

static uint8_t _cmbHopSeq[CMB_ELRS_CHANNELS];
static const uint8_t CMB_PAYLOAD[] = { 0xE1, 0x25, 0x00, 0x00 };

static void cmbBuildHopSequence() {
    for (uint8_t i = 0; i < CMB_ELRS_CHANNELS; i++) _cmbHopSeq[i] = i;
    uint32_t rng = 0xFEEDFACE;
    for (uint8_t i = CMB_ELRS_CHANNELS - 1; i > 0; i--) {
        rng = rng * 1664525UL + 1013904223UL;
        uint8_t j = rng % (i + 1);
        uint8_t tmp = _cmbHopSeq[i];
        _cmbHopSeq[i] = _cmbHopSeq[j];
        _cmbHopSeq[j] = tmp;
    }
}

// FreeRTOS task running ELRS FHSS on Core 1
static void elrsTask(void *param) {
    SX1262 *radio = (SX1262 *)param;
    uint8_t hopIdx = 0;

    // Configure radio for ELRS mode
    radio->standby();
    float freq = CMB_ELRS_START + (_cmbHopSeq[0] * CMB_ELRS_SPACING);
    radio->begin(freq, 500.0, 6, 5,
                 RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                 rfGetPower(), 8, 0, false);
    radio->explicitHeader();

    unsigned long lastHopUs = micros();

    while (_elrsTaskRunning) {
        unsigned long nowUs = micros();
        if ((nowUs - lastHopUs) >= CMB_HOP_US) {
            lastHopUs = nowUs;

            hopIdx = (hopIdx + 1) % CMB_ELRS_CHANNELS;
            freq = CMB_ELRS_START + (_cmbHopSeq[hopIdx] * CMB_ELRS_SPACING);
            radio->setFrequency(freq);
            radio->transmit(CMB_PAYLOAD, sizeof(CMB_PAYLOAD));

            _elrsTaskPkts++;
            _elrsTaskHops++;
        }

        // Yield briefly to prevent watchdog trigger
        vTaskDelay(1);
    }

    radio->standby();
    radio->begin(915.0);  // reset to default config
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
