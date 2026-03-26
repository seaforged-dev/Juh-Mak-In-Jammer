#ifndef RID_SPOOFER_H
#define RID_SPOOFER_H

#include <Arduino.h>

// ============================================================
// Mode 1: Remote ID Spoofer
// Broadcasts fake ASTM F3411 Open Drone ID via WiFi beacons
// and BLE 4 Legacy Advertising simultaneously.
// ============================================================

// Drone identity and position for spoofing
struct DroneState {
    // Identity
    char serialNumber[21];    // ASTM F3411 serial (up to 20 chars + null)
    uint8_t uaType;           // 0=None, 1=Aeroplane, 2=Helicopter/Multirotor, etc.

    // Position
    double latitude;          // degrees
    double longitude;         // degrees
    float altGeo;             // meters above WGS84 ellipsoid
    float altBaro;            // meters barometric
    float height;             // meters above ground
    float speed;              // m/s horizontal
    float vspeed;             // m/s vertical
    float heading;            // degrees from north

    // Operator
    double opLat;             // operator latitude
    double opLon;             // operator longitude
    char operatorId[21];      // operator registration ID
};

// Status snapshot for OLED
struct RidParams {
    uint32_t wifiPackets;
    uint32_t blePackets;
    double latitude;
    double longitude;
    float altitude;
    bool running;
};

// --- Public API ---
void ridInit();
void ridStart();
void ridStop();
void ridUpdate();            // call every loop() — handles TX timing
RidParams ridGetParams();

#endif // RID_SPOOFER_H
