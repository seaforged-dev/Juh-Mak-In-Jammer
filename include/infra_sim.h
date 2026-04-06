#ifndef INFRA_SIM_H
#define INFRA_SIM_H

#include <RadioLib.h>

// ============================================================
// Infrastructure False Positive Simulation Modes
// v2 ref §4.2, §4.3, §4.4 — Meshtastic, Helium PoC, LoRaWAN EU868
// These must NOT trigger SENTRY-RF WARNING/CRITICAL alerts
// ============================================================

enum InfraMode {
    INFRA_MESHTASTIC = 0,   // f1: Meshtastic beacon
    INFRA_HELIUM_POC = 1,   // f2: Helium PoC multi-hotspot
    INFRA_LORAWAN_EU = 2,   // f3: LoRaWAN EU868
};

struct InfraParams {
    InfraMode mode;
    float    lastFreqMHz;
    uint8_t  lastSF;
    uint32_t packetCount;
    uint32_t elapsedSec;
    int8_t   powerDbm;
    bool     running;
};

void infraInit(SX1262 *radio);
void infraStart(InfraMode mode);
void infraStop();
void infraUpdate();         // call every loop()
InfraParams infraGetParams();

#endif // INFRA_SIM_H
