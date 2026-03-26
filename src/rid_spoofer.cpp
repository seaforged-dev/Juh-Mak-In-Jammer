#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include "rid_spoofer.h"

// ============================================================
// ASTM F3411 Open Drone ID — Manual Message Encoding
// ============================================================
// Each ODID message is 25 bytes: 1 byte header + 24 bytes payload.
// Header: [msg_type(4 bits) | proto_version(4 bits)]

static constexpr uint8_t ODID_MSG_TYPE_BASIC_ID   = 0x00;
static constexpr uint8_t ODID_MSG_TYPE_LOCATION    = 0x10;
static constexpr uint8_t ODID_MSG_TYPE_SYSTEM      = 0x40;
static constexpr uint8_t ODID_MSG_TYPE_OPERATOR_ID = 0x50;
static constexpr uint8_t ODID_PROTO_VERSION        = 0x02;  // F3411-22a
static constexpr uint8_t ODID_MSG_SIZE             = 25;

// Message pack containing multiple ODID messages
static constexpr uint8_t ODID_MSG_PACK_TYPE        = 0xF0;
static constexpr uint8_t ODID_MSG_PACK_MAX_MSGS    = 4;

// WiFi vendor-specific element OUI for ODID (CTA-2063-A)
static const uint8_t ODID_WIFI_OUI[] = { 0xFA, 0x0B, 0xBC };
static constexpr uint8_t ODID_WIFI_OUI_TYPE = 0x0D;

// ============================================================
// State
// ============================================================

static bool _ridRunning = false;
static DroneState _drone;
static uint32_t _wifiCount = 0;
static uint32_t _bleCount = 0;
static bool _wifiReady = false;
static bool _bleReady = false;

// TX timing — send one burst per second (WiFi + BLE)
static unsigned long _lastTxMs = 0;
static constexpr unsigned long RID_TX_INTERVAL_MS = 1000;

// Random MAC for WiFi beacon — changes per boot
static uint8_t _beaconMac[6];

// ============================================================
// ODID Message Encoding
// ============================================================

static int32_t encodeLatLon(double deg) {
    return (int32_t)(deg * 1e7);
}

static uint16_t encodeAlt(float meters) {
    // ODID altitude: offset by -1000m, in 0.5m increments
    return (uint16_t)((meters + 1000.0f) * 2.0f);
}

static uint8_t encodeSpeed(float mps) {
    // ODID speed: 0.25 m/s multiplier for < 63.75, else use high range
    if (mps < 0) mps = 0;
    if (mps <= 63.75f) return (uint8_t)(mps * 4.0f);
    return 0xFF;  // max
}

static void buildBasicIdMsg(uint8_t *buf) {
    memset(buf, 0, ODID_MSG_SIZE);
    buf[0] = ODID_MSG_TYPE_BASIC_ID | ODID_PROTO_VERSION;
    // Byte 1: [id_type(4) | ua_type(4)]
    // id_type=1 (Serial Number), ua_type from drone state
    buf[1] = (0x01 << 4) | (_drone.uaType & 0x0F);
    // Bytes 2-21: Serial number (20 chars, padded with nulls)
    strncpy((char *)&buf[2], _drone.serialNumber, 20);
}

static void buildLocationMsg(uint8_t *buf) {
    memset(buf, 0, ODID_MSG_SIZE);
    buf[0] = ODID_MSG_TYPE_LOCATION | ODID_PROTO_VERSION;
    // Byte 1: [status(4) | height_type(1) | ew_dir(1) | speed_mult(1) | reserved(1)]
    // status=2 (Airborne), height_type=0 (above takeoff), ew_dir=0, speed_mult=0
    buf[1] = 0x20;
    // Byte 2: Direction (0-360 mapped to 0-255... but actually it's in 0-359 degrees)
    buf[2] = (uint8_t)(_drone.heading / 360.0f * 255.0f);
    // Byte 3: Speed (horizontal)
    buf[3] = encodeSpeed(_drone.speed);
    // Byte 4: Vertical speed (in 0.5 m/s, signed, offset by 63)
    buf[4] = (uint8_t)((int8_t)(_drone.vspeed * 2.0f) + 63);
    // Bytes 5-8: Latitude (int32 LE, degrees * 1e7)
    int32_t lat = encodeLatLon(_drone.latitude);
    memcpy(&buf[5], &lat, 4);
    // Bytes 9-12: Longitude (int32 LE)
    int32_t lon = encodeLatLon(_drone.longitude);
    memcpy(&buf[9], &lon, 4);
    // Bytes 13-14: Pressure altitude (uint16 LE)
    uint16_t altBaro = encodeAlt(_drone.altBaro);
    memcpy(&buf[13], &altBaro, 2);
    // Bytes 15-16: Geodetic altitude (uint16 LE)
    uint16_t altGeo = encodeAlt(_drone.altGeo);
    memcpy(&buf[15], &altGeo, 2);
    // Bytes 17-18: Height AGL (uint16 LE)
    uint16_t height = encodeAlt(_drone.height);
    memcpy(&buf[17], &height, 2);
    // Bytes 19-20: Horizontal/Vertical accuracy (0 = unknown)
    // Bytes 21-22: Baro alt accuracy, timestamp
    // Byte 23: Timestamp adjustment (0.1s units since last full hour)
    uint32_t secSinceBoot = millis() / 1000;
    uint16_t secSinceHour = (uint16_t)(secSinceBoot % 3600);
    buf[23] = (uint8_t)((secSinceHour % 60) * 10);  // 0.1s within minute
}

static void buildSystemMsg(uint8_t *buf) {
    memset(buf, 0, ODID_MSG_SIZE);
    buf[0] = ODID_MSG_TYPE_SYSTEM | ODID_PROTO_VERSION;
    // Byte 1: [operator_location_type(2) | classification_type(3) | reserved(3)]
    buf[1] = 0x00;  // takeoff location, undeclared
    // Bytes 2-5: Operator latitude
    int32_t lat = encodeLatLon(_drone.opLat);
    memcpy(&buf[2], &lat, 4);
    // Bytes 6-9: Operator longitude
    int32_t lon = encodeLatLon(_drone.opLon);
    memcpy(&buf[6], &lon, 4);
    // Bytes 10-11: Area count + radius
    buf[10] = 1;   // area count
    buf[11] = 0;   // area radius (0 = point)
}

static void buildOperatorIdMsg(uint8_t *buf) {
    memset(buf, 0, ODID_MSG_SIZE);
    buf[0] = ODID_MSG_TYPE_OPERATOR_ID | ODID_PROTO_VERSION;
    buf[1] = 0x00;  // operator ID type = CAA
    strncpy((char *)&buf[2], _drone.operatorId, 20);
}

// ============================================================
// WiFi Beacon Frame Construction
// ============================================================

// Max frame size: MAC header(24) + fixed(12) + SSID(2) + vendor IE overhead(7)
//                + message pack header(3) + 4 messages(100) + FCS handled by HW
static constexpr size_t BEACON_MAX_LEN = 256;
static uint8_t _beaconFrame[BEACON_MAX_LEN];

static size_t buildBeaconFrame() {
    size_t pos = 0;

    // --- 802.11 MAC Header ---
    // Frame control: 0x80 0x00 = Beacon
    _beaconFrame[pos++] = 0x80;
    _beaconFrame[pos++] = 0x00;
    // Duration
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;
    // Destination: broadcast
    memset(&_beaconFrame[pos], 0xFF, 6); pos += 6;
    // Source: random MAC
    memcpy(&_beaconFrame[pos], _beaconMac, 6); pos += 6;
    // BSSID: same as source
    memcpy(&_beaconFrame[pos], _beaconMac, 6); pos += 6;
    // Sequence control (will be filled by HW)
    _beaconFrame[pos++] = 0x00;
    _beaconFrame[pos++] = 0x00;

    // --- Beacon Frame Body ---
    // Timestamp (8 bytes, filled by HW or zeros)
    memset(&_beaconFrame[pos], 0, 8); pos += 8;
    // Beacon interval: 100 TU (102.4 ms)
    _beaconFrame[pos++] = 0x64;
    _beaconFrame[pos++] = 0x00;
    // Capability info
    _beaconFrame[pos++] = 0x01;
    _beaconFrame[pos++] = 0x00;

    // --- SSID IE (hidden / zero-length) ---
    _beaconFrame[pos++] = 0x00;  // element ID = SSID
    _beaconFrame[pos++] = 0x00;  // length = 0

    // --- Vendor Specific IE with ODID Message Pack ---
    size_t vendorIeStart = pos;
    _beaconFrame[pos++] = 0xDD;  // element ID = Vendor Specific
    size_t vendorLenPos = pos;
    _beaconFrame[pos++] = 0x00;  // length placeholder

    // OUI + type
    memcpy(&_beaconFrame[pos], ODID_WIFI_OUI, 3); pos += 3;
    _beaconFrame[pos++] = ODID_WIFI_OUI_TYPE;

    // Message Pack header
    _beaconFrame[pos++] = ODID_MSG_PACK_TYPE | ODID_PROTO_VERSION;
    _beaconFrame[pos++] = ODID_MSG_SIZE;  // single message size
    _beaconFrame[pos++] = ODID_MSG_PACK_MAX_MSGS;  // message count

    // Encode all 4 messages into the pack
    buildBasicIdMsg(&_beaconFrame[pos]);    pos += ODID_MSG_SIZE;
    buildLocationMsg(&_beaconFrame[pos]);   pos += ODID_MSG_SIZE;
    buildSystemMsg(&_beaconFrame[pos]);     pos += ODID_MSG_SIZE;
    buildOperatorIdMsg(&_beaconFrame[pos]); pos += ODID_MSG_SIZE;

    // Fill in vendor IE length (everything after the length byte)
    _beaconFrame[vendorLenPos] = (uint8_t)(pos - vendorLenPos - 1);

    // FCS is auto-calculated by esp_wifi_80211_tx — do NOT append it
    return pos;
}

// ============================================================
// WiFi Init — needed for raw frame TX
// ============================================================

static void wifiInit() {
    // NVS is needed by WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // Enable long-range mode for better TX
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    _wifiReady = true;
    Serial.println("RID: WiFi initialized for beacon TX");
}

static void wifiTransmitBeacon() {
    if (!_wifiReady) return;

    size_t len = buildBeaconFrame();
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, _beaconFrame, len, true);
    if (err == ESP_OK) {
        _wifiCount++;
    } else {
        Serial.printf("RID: WiFi TX error %d\n", err);
    }
}

// ============================================================
// BLE Advertising with ODID payload
// ============================================================

static uint8_t _bleAdvData[31];  // max BLE 4 adv payload

static void bleGapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    // We don't need to handle any events for advertising-only mode
    (void)event;
    (void)param;
}

static void bleInit() {
    // Release classic BT memory — we only need BLE
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gap_register_callback(bleGapCallback);

    _bleReady = true;
    Serial.println("RID: BLE initialized for advertising");
}

static void bleTransmitOdid() {
    if (!_bleReady) return;

    // Stop any current advertising before updating data
    esp_ble_gap_stop_advertising();

    // Build BLE advertising data with ODID Basic ID
    // BLE ODID format: AD struct with type 0x16 (Service Data)
    // Service UUID for ODID: 0xFFFA (ASTM)
    size_t pos = 0;

    // Flags AD struct (required for discoverable mode)
    _bleAdvData[pos++] = 0x02;  // length
    _bleAdvData[pos++] = 0x01;  // type: flags
    _bleAdvData[pos++] = 0x06;  // LE General Discoverable + BR/EDR Not Supported

    // Service Data AD struct with ODID message
    uint8_t svcDataLen = 2 + 1 + ODID_MSG_SIZE;  // UUID(2) + msg_counter(1) + message(25)
    _bleAdvData[pos++] = svcDataLen + 1;  // AD length (includes type byte)
    _bleAdvData[pos++] = 0x16;            // type: Service Data - 16-bit UUID
    _bleAdvData[pos++] = 0xFA;            // UUID low byte (0xFFFA)
    _bleAdvData[pos++] = 0xFF;            // UUID high byte
    _bleAdvData[pos++] = 0x0D;            // ODID app code + message counter

    // Alternate between Basic ID and Location messages each cycle
    static bool sendBasicId = true;
    if (sendBasicId) {
        buildBasicIdMsg(&_bleAdvData[pos]);
    } else {
        buildLocationMsg(&_bleAdvData[pos]);
    }
    pos += ODID_MSG_SIZE;
    sendBasicId = !sendBasicId;

    esp_ble_gap_config_adv_data_raw(_bleAdvData, pos);

    // Start advertising with fast interval
    esp_ble_adv_params_t adv_params = {};
    adv_params.adv_int_min = 0x20;  // 20ms
    adv_params.adv_int_max = 0x40;  // 40ms
    adv_params.adv_type = ADV_TYPE_NONCONN_IND;  // non-connectable
    adv_params.own_addr_type = BLE_ADDR_TYPE_RANDOM;
    adv_params.channel_map = ADV_CHNL_ALL;

    esp_ble_gap_start_advertising(&adv_params);
    _bleCount++;
}

// ============================================================
// Public API
// ============================================================

void ridInit() {
    _ridRunning = false;
    _wifiCount = 0;
    _bleCount = 0;

    // Generate random beacon MAC (locally administered, unicast)
    esp_fill_random(_beaconMac, 6);
    _beaconMac[0] = (_beaconMac[0] | 0x02) & 0xFE;  // set local bit, clear multicast

    // Set up default drone identity
    strncpy(_drone.serialNumber, "JMKN-TEST-00001", sizeof(_drone.serialNumber));
    _drone.uaType = 2;  // Helicopter/Multirotor

    // Default position: Virginia Beach area (near SENTRY-RF test site)
    _drone.latitude  = 36.8529;
    _drone.longitude = -75.978;
    _drone.altGeo    = 120.0f;
    _drone.altBaro   = 119.5f;
    _drone.height    = 50.0f;
    _drone.speed     = 5.0f;
    _drone.vspeed    = 0.0f;
    _drone.heading   = 90.0f;

    // Operator position (slightly offset from drone)
    _drone.opLat  = 36.8520;
    _drone.opLon  = -75.979;
    strncpy(_drone.operatorId, "OP-JMKN-TEST", sizeof(_drone.operatorId));
}

void ridStart() {
    if (!_wifiReady) wifiInit();
    if (!_bleReady)  bleInit();

    _wifiCount = 0;
    _bleCount = 0;
    _ridRunning = true;
    _lastTxMs = millis();

    Serial.println("RID: Spoofing started (WiFi beacons + BLE adverts)");
    Serial.printf("RID: Drone ID=%s, Pos=%.4f,%.4f, Alt=%.0fm\n",
                  _drone.serialNumber, _drone.latitude, _drone.longitude, _drone.altGeo);
}

void ridStop() {
    _ridRunning = false;

    if (_bleReady) {
        esp_ble_gap_stop_advertising();
    }

    Serial.printf("RID: Stopped — %lu WiFi beacons, %lu BLE adverts\n",
                  (unsigned long)_wifiCount, (unsigned long)_bleCount);
}

void ridUpdate() {
    if (!_ridRunning) return;

    unsigned long now = millis();
    if ((now - _lastTxMs) < RID_TX_INTERVAL_MS) return;
    _lastTxMs = now;

    // Transmit WiFi beacon with all 4 ODID messages in a message pack
    wifiTransmitBeacon();

    // Transmit BLE advertisement (alternates Basic ID / Location)
    bleTransmitOdid();
}

RidParams ridGetParams() {
    return RidParams{
        _wifiCount,
        _bleCount,
        _drone.latitude,
        _drone.longitude,
        _drone.altGeo,
        _ridRunning,
    };
}
