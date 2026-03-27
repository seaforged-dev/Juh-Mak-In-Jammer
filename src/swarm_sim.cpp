#include <Arduino.h>
#include <esp_wifi.h>
#include <math.h>
#include "swarm_sim.h"

// ============================================================
// Drone Swarm Simulator — multiple virtual drones broadcasting
// ASTM F3411 Remote ID WiFi beacons in round-robin
// ============================================================

// ODID constants (same as rid_spoofer)
static constexpr uint8_t ODID_MSG_TYPE_BASIC_ID   = 0x00;
static constexpr uint8_t ODID_MSG_TYPE_LOCATION    = 0x10;
static constexpr uint8_t ODID_MSG_TYPE_SYSTEM      = 0x40;
static constexpr uint8_t ODID_MSG_TYPE_OPERATOR_ID = 0x50;
static constexpr uint8_t ODID_PROTO_VERSION        = 0x02;
static constexpr uint8_t ODID_MSG_SIZE             = 25;
static constexpr uint8_t ODID_MSG_PACK_TYPE        = 0xF0;
static const uint8_t ODID_WIFI_OUI[] = { 0xFA, 0x0B, 0xBC };
static constexpr uint8_t ODID_WIFI_OUI_TYPE = 0x0D;

// ── Virtual Drone State ─────────────────────────────────────

struct VirtualDrone {
    char     uasId[21];       // unique ID string
    uint8_t  mac[6];          // unique WiFi beacon MAC
    double   centerLat;       // orbit center
    double   centerLon;
    float    radiusM;         // orbit radius in meters
    float    altitudeM;       // base altitude
    float    speedMps;        // ground speed
    float    phaseRad;        // current angular position
    float    phaseRate;       // radians per second (derived from speed/radius)
    float    altWobble;       // current altitude offset

    // Current computed position
    double   lat;
    double   lon;
    float    alt;
    float    heading;
    float    speed;
};

// ── State ───────────────────────────────────────────────────

static VirtualDrone _drones[SWARM_MAX_DRONES];
static uint8_t  _droneCount    = 4;
static uint8_t  _txIndex       = 0;    // round-robin beacon index
static bool     _running       = false;
static bool     _wifiReady     = false;
static uint32_t _beaconCount   = 0;
static unsigned long _startMs  = 0;
static unsigned long _lastUpdateMs = 0;
static unsigned long _lastTxMs = 0;

static constexpr unsigned long UPDATE_INTERVAL_MS = 100;  // position update rate
static constexpr unsigned long TX_INTERVAL_MS     = 50;   // beacon TX rate

// Beacon frame buffer
static constexpr size_t BEACON_MAX_LEN = 256;
static uint8_t _beaconFrame[BEACON_MAX_LEN];

// ── Helpers ─────────────────────────────────────────────────

static constexpr double DEG_PER_METER_LAT = 1.0 / 111320.0;

static double degPerMeterLon(double lat) {
    return 1.0 / (111320.0 * cos(lat * M_PI / 180.0));
}

static int32_t encodeLatLon(double deg) {
    return (int32_t)(deg * 1e7);
}

static uint16_t encodeAlt(float meters) {
    return (uint16_t)((meters + 1000.0f) * 2.0f);
}

static uint8_t encodeSpeed(float mps) {
    if (mps < 0) mps = 0;
    if (mps <= 63.75f) return (uint8_t)(mps * 4.0f);
    return 0xFF;
}

// ── Drone Initialization ────────────────────────────────────

static void initDrone(VirtualDrone& d, uint8_t index) {
    // Unique ID
    snprintf(d.uasId, sizeof(d.uasId), "SWRM-DRONE-%02u", index + 1);

    // Unique MAC (locally administered)
    esp_fill_random(d.mac, 6);
    d.mac[0] = (d.mac[0] | 0x02) & 0xFE;  // locally administered, unicast
    d.mac[5] = index;  // ensure uniqueness

    // Base position: Virginia Beach area with slight offset per drone
    d.centerLat = 36.8529 + (index * 0.001);
    d.centerLon = -75.9780 + (index * 0.001);

    // Randomized flight parameters
    d.radiusM   = 50.0f + (esp_random() % 250);     // 50-300m
    d.altitudeM = 30.0f + (esp_random() % 90);      // 30-120m
    d.speedMps  = 5.0f + (esp_random() % 10);       // 5-15 m/s

    // Phase offset so drones don't start at same position
    d.phaseRad  = (float)(index * 2.0 * M_PI / 16.0);
    d.phaseRate = d.speedMps / d.radiusM;  // omega = v/r

    d.altWobble = 0;
    d.lat = d.centerLat;
    d.lon = d.centerLon;
    d.alt = d.altitudeM;
    d.heading = 0;
    d.speed = d.speedMps;
}

// ── Position Update ─────────────────────────────────────────

static void updateDronePosition(VirtualDrone& d, float dt) {
    // Advance phase along circular orbit
    // Add ±5% speed variation
    float speedVar = 1.0f + ((int32_t)(esp_random() % 100) - 50) * 0.001f;
    d.phaseRad += d.phaseRate * dt * speedVar;
    if (d.phaseRad > 2.0f * M_PI) d.phaseRad -= 2.0f * M_PI;

    // Compute position on circle
    float offsetN = d.radiusM * cosf(d.phaseRad);  // meters north
    float offsetE = d.radiusM * sinf(d.phaseRad);  // meters east

    d.lat = d.centerLat + offsetN * DEG_PER_METER_LAT;
    d.lon = d.centerLon + offsetE * degPerMeterLon(d.centerLat);

    // Altitude wobble: ±2m random walk
    d.altWobble += ((int32_t)(esp_random() % 100) - 50) * 0.02f;
    if (d.altWobble > 2.0f) d.altWobble = 2.0f;
    if (d.altWobble < -2.0f) d.altWobble = -2.0f;
    d.alt = d.altitudeM + d.altWobble;

    // Heading: tangent to circle (perpendicular to radius)
    d.heading = fmodf((d.phaseRad * 180.0f / M_PI) + 90.0f + 360.0f, 360.0f);
    d.speed = d.speedMps * speedVar;
}

// ── Beacon Frame Builder ────────────────────────────────────

static size_t buildSwarmBeacon(const VirtualDrone& d) {
    size_t pos = 0;

    // MAC header: beacon frame
    _beaconFrame[pos++] = 0x80;
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;
    memset(&_beaconFrame[pos], 0xFF, 6); pos += 6;   // dst: broadcast
    memcpy(&_beaconFrame[pos], d.mac, 6); pos += 6;  // src: drone MAC
    memcpy(&_beaconFrame[pos], d.mac, 6); pos += 6;  // bssid
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;

    // Fixed fields
    memset(&_beaconFrame[pos], 0, 8); pos += 8;  // timestamp
    _beaconFrame[pos++] = 0x64; _beaconFrame[pos++] = 0x00;  // interval
    _beaconFrame[pos++] = 0x01; _beaconFrame[pos++] = 0x00;  // capability

    // SSID IE (empty)
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;

    // Vendor-specific IE with ODID
    _beaconFrame[pos++] = 0xDD;
    size_t lenPos = pos;
    _beaconFrame[pos++] = 0x00;  // length placeholder

    memcpy(&_beaconFrame[pos], ODID_WIFI_OUI, 3); pos += 3;
    _beaconFrame[pos++] = ODID_WIFI_OUI_TYPE;

    // Message pack header
    _beaconFrame[pos++] = ODID_MSG_PACK_TYPE | ODID_PROTO_VERSION;
    _beaconFrame[pos++] = ODID_MSG_SIZE;
    _beaconFrame[pos++] = 2;  // 2 messages: Basic ID + Location

    // Basic ID message
    uint8_t* msg = &_beaconFrame[pos];
    memset(msg, 0, ODID_MSG_SIZE);
    msg[0] = ODID_MSG_TYPE_BASIC_ID | ODID_PROTO_VERSION;
    msg[1] = (0x01 << 4) | 0x02;  // Serial Number, Multirotor
    strncpy((char*)&msg[2], d.uasId, 20);
    pos += ODID_MSG_SIZE;

    // Location message
    msg = &_beaconFrame[pos];
    memset(msg, 0, ODID_MSG_SIZE);
    msg[0] = ODID_MSG_TYPE_LOCATION | ODID_PROTO_VERSION;
    msg[1] = 0x20;  // airborne
    msg[2] = (uint8_t)(d.heading / 360.0f * 255.0f);
    msg[3] = encodeSpeed(d.speed);
    msg[4] = 63;  // vspeed = 0
    int32_t lat = encodeLatLon(d.lat);
    memcpy(&msg[5], &lat, 4);
    int32_t lon = encodeLatLon(d.lon);
    memcpy(&msg[9], &lon, 4);
    uint16_t altVal = encodeAlt(d.alt);
    memcpy(&msg[13], &altVal, 2);
    memcpy(&msg[15], &altVal, 2);
    memcpy(&msg[17], &altVal, 2);
    pos += ODID_MSG_SIZE;

    _beaconFrame[lenPos] = (uint8_t)(pos - lenPos - 1);
    return pos;
}

// ── WiFi Init ───────────────────────────────────────────────

static void swarmWifiInit() {
    if (_wifiReady) return;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    _wifiReady = true;
}

// ── Public API ──────────────────────────────────────────────

void swarmInit() {
    _running = false;
    _droneCount = 4;
}

void swarmStart(uint8_t count) {
    _droneCount = count;
    if (_droneCount > SWARM_MAX_DRONES) _droneCount = SWARM_MAX_DRONES;
    if (_droneCount < 1) _droneCount = 1;

    for (uint8_t i = 0; i < _droneCount; i++) {
        initDrone(_drones[i], i);
    }

    swarmWifiInit();

    _txIndex = 0;
    _beaconCount = 0;
    _startMs = millis();
    _lastUpdateMs = millis();
    _lastTxMs = millis();
    _running = true;

    Serial.printf("SWARM: %d drones active, broadcasting RID beacons\n", _droneCount);
    for (uint8_t i = 0; i < _droneCount; i++) {
        Serial.printf("  [%02d] %s  r=%.0fm alt=%.0fm spd=%.0fm/s\n",
                      i + 1, _drones[i].uasId,
                      _drones[i].radiusM, _drones[i].altitudeM, _drones[i].speedMps);
    }
}

void swarmStop() {
    _running = false;
    Serial.printf("SWARM: Stopped after %lu beacons\n", (unsigned long)_beaconCount);
}

void swarmUpdate() {
    if (!_running) return;

    unsigned long now = millis();

    // Update all drone positions at 10 Hz
    if (now - _lastUpdateMs >= UPDATE_INTERVAL_MS) {
        float dt = (now - _lastUpdateMs) / 1000.0f;
        _lastUpdateMs = now;
        for (uint8_t i = 0; i < _droneCount; i++) {
            updateDronePosition(_drones[i], dt);
        }
    }

    // Transmit one beacon per TX interval (round-robin through drones)
    if (now - _lastTxMs >= TX_INTERVAL_MS) {
        _lastTxMs = now;

        size_t len = buildSwarmBeacon(_drones[_txIndex]);
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, _beaconFrame, len, true);
        if (err == ESP_OK) {
            _beaconCount++;
        }

        _txIndex = (_txIndex + 1) % _droneCount;
    }
}

void swarmCycleCount() {
    static const uint8_t presets[] = { 1, 4, 8, 16 };
    static uint8_t idx = 1;  // start at 4
    idx = (idx + 1) % 4;
    uint8_t newCount = presets[idx];

    if (_running) {
        swarmStop();
        swarmStart(newCount);
    } else {
        _droneCount = newCount;
    }
    Serial.printf("SWARM: drone count set to %d\n", newCount);
}

SwarmParams swarmGetParams() {
    return SwarmParams{
        _droneCount,
        _beaconCount,
        _running ? (uint32_t)((millis() - _startMs) / 1000) : 0,
        _running,
    };
}
