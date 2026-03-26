#ifndef COMBINED_MODE_H
#define COMBINED_MODE_H

#include <RadioLib.h>

// ============================================================
// Mode 4: Combined Attack
// Runs Remote ID (WiFi+BLE) on Core 0 and ELRS FHSS (SX1262)
// on Core 1 simultaneously via FreeRTOS tasks.
// ============================================================

struct CombinedParams {
    uint32_t ridWifiPkts;
    uint32_t ridBlePkts;
    uint32_t elrsPkts;
    uint32_t elrsHops;
    uint32_t elapsedSec;
    bool running;
};

// --- Public API ---
void combinedInit(SX1262 *radio);
void combinedStart();
void combinedStop();
void combinedUpdate();           // call from loop() — updates RID on Core 0
CombinedParams combinedGetParams();

#endif // COMBINED_MODE_H
