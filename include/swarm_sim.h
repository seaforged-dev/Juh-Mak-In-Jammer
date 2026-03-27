#ifndef SWARM_SIM_H
#define SWARM_SIM_H

#include <cstdint>

// ============================================================
// Drone Swarm Simulator — multiple virtual drones broadcasting
// ASTM F3411 Remote ID WiFi beacons in round-robin
// ============================================================

static const int SWARM_MAX_DRONES = 16;

struct SwarmParams {
    uint8_t  droneCount;
    uint32_t beaconCount;
    uint32_t elapsedSec;
    bool     running;
};

void swarmInit();
void swarmStart(uint8_t count = 4);
void swarmStop();
void swarmUpdate();           // call every loop() iteration
void swarmCycleCount();       // cycle drone count: 1→4→8→16
SwarmParams swarmGetParams();

#endif // SWARM_SIM_H
