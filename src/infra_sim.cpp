#include <Arduino.h>
#include "infra_sim.h"
#include "rf_modes.h"       // for rfGetPower()
#include "protocol_params.h"

// ============================================================
// Infrastructure False Positive Simulation — v2 ref §4.2-4.4
// ============================================================
// These modes simulate non-drone LoRa sources that SENTRY-RF
// must correctly classify as infrastructure (not drone FHSS).

static SX1262 *_radio = nullptr;
static InfraMode _mode = INFRA_MESHTASTIC;
static bool     _running     = false;
static uint32_t _packetCount = 0;
static float    _lastFreq    = 0;
static uint8_t  _lastSF      = 7;
static unsigned long _startMs = 0;

// Simple PRNG for randomized timing
static uint32_t _rng = 0xABCD1234;
static uint32_t rng() { _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5; return _rng; }
static uint32_t rngRange(uint32_t lo, uint32_t hi) { return lo + (rng() % (hi - lo + 1)); }

// Dummy payload
static uint8_t _payload[30];
static void fillRandomPayload(uint8_t len) {
    for (uint8_t i = 0; i < len && i < sizeof(_payload); i++)
        _payload[i] = (uint8_t)(rng() & 0xFF);
}

// ============================================================
// MODE 1: Meshtastic Beacon — v2 ref §4.3
// ============================================================
// Fixed frequency, 16-sym preamble, sync word 0x2B
// Beacon every ~15 minutes + occasional burst cascades

static const float    MESH_FREQ       = 906.875f;  // US Long-Fast default
static const uint8_t  MESH_SF         = 11;         // Long-Fast preset
static const float    MESH_BW_KHZ     = 250.0f;     // Long-Fast preset
static unsigned long  _meshNextBeaconMs = 0;
static bool           _meshBurstActive = false;
static uint8_t        _meshBurstRemaining = 0;
static unsigned long  _meshBurstNextMs = 0;

static void meshConfigRadio() {
    _radio->standby();
    _radio->begin(
        MESH_FREQ, MESH_BW_KHZ, MESH_SF, 5,
        SYNC_WORD_MESHTASTIC,     // 0x2B — v2 §4.3
        rfGetPower(),
        MESHTASTIC_PREAMBLE_LEN,  // 16 symbols — v2 §4.3
        1.8, false
    );
    _radio->explicitHeader();
}

static void meshTransmit() {
    meshConfigRadio();
    fillRandomPayload(20);
    _lastFreq = MESH_FREQ;
    _lastSF = MESH_SF;
    _radio->transmit(_payload, 20);
    _packetCount++;
}

static void meshStart() {
    _meshNextBeaconMs = millis() + 5000;  // first beacon after 5s (not 15 min for testing)
    _meshBurstActive = false;

    Serial.printf("[Meshtastic] %.3fMHz SF%u/BW%.0f 16sym preamble sync=0x%02X\n",
                  MESH_FREQ, MESH_SF, MESH_BW_KHZ, SYNC_WORD_MESHTASTIC);
    Serial.println("  Pattern: beacon every ~15min + burst cascades");
    Serial.println("  SENTRY-RF should: CLEAR or ADVISORY (no diversity)");
}

static void meshUpdate() {
    unsigned long now = millis();

    // Handle burst cascade (3-5 packets in ~2 seconds, simulating mesh relay)
    if (_meshBurstActive) {
        if (now >= _meshBurstNextMs && _meshBurstRemaining > 0) {
            meshTransmit();
            _meshBurstRemaining--;
            _meshBurstNextMs = now + rngRange(300, 600);  // 300-600ms between relay hops
            Serial.printf("  Mesh relay: pkt %u of burst\n", _packetCount);
            if (_meshBurstRemaining == 0) {
                _meshBurstActive = false;
                // Next beacon in 60-120s (shortened from 15 min for testing)
                _meshNextBeaconMs = now + rngRange(60000, 120000);
            }
        }
        return;
    }

    // Periodic beacon
    if (now >= _meshNextBeaconMs) {
        meshTransmit();
        Serial.printf("  Mesh beacon: pkt %u @ %.3f MHz\n", _packetCount, MESH_FREQ);

        // 30% chance of triggering a burst cascade (simulating message relay)
        if ((rng() % 100) < 30) {
            _meshBurstActive = true;
            _meshBurstRemaining = (uint8_t)rngRange(3, 5);
            _meshBurstNextMs = now + rngRange(300, 600);
            Serial.printf("  Mesh cascade triggered: %u relay packets\n", _meshBurstRemaining);
        } else {
            _meshNextBeaconMs = now + rngRange(60000, 120000);
        }
    }
}

// ============================================================
// MODE 2: Helium PoC Beacons — v2 ref §4.4
// ============================================================
// 5 simulated hotspots, each beaconing ~1/30s on SB2 channels

static const uint8_t HELIUM_NUM_HOTSPOTS = 5;

struct HeliumHotspot {
    unsigned long nextTxMs;
    uint8_t       lastChIdx;
};

static HeliumHotspot _hotspots[HELIUM_NUM_HOTSPOTS];

static void heliumConfigRadio(float freq, uint8_t sf) {
    _radio->standby();
    _radio->begin(
        freq, 125.0, sf, 5,
        SYNC_WORD_LORAWAN,      // 0x34 — v2 §4.4
        rfGetPower(),
        LORAWAN_PREAMBLE_LEN,   // 8 symbols
        1.8, false
    );
    _radio->explicitHeader();
}

static void heliumStart() {
    unsigned long now = millis();
    // Stagger hotspot start times
    for (uint8_t i = 0; i < HELIUM_NUM_HOTSPOTS; i++) {
        _hotspots[i].nextTxMs = now + (i * 6000) + rngRange(1000, 3000);
        _hotspots[i].lastChIdx = rng() % LORAWAN_US915_SB2_COUNT;
    }

    Serial.printf("[Helium PoC] %u hotspots, %u SB2 channels, ~10 beacons/min\n",
                  HELIUM_NUM_HOTSPOTS, LORAWAN_US915_SB2_COUNT);
    Serial.println("  Pattern: staggered beacons on rotating channels");
    Serial.println("  SENTRY-RF should: CLEAR/ADVISORY (slow diversity, no velocity)");
}

static void heliumUpdate() {
    unsigned long now = millis();

    for (uint8_t i = 0; i < HELIUM_NUM_HOTSPOTS; i++) {
        if (now >= _hotspots[i].nextTxMs) {
            // Pick random SB2 channel and SF
            uint8_t chIdx = rng() % LORAWAN_US915_SB2_COUNT;
            uint8_t sf = (uint8_t)rngRange(8, 10);
            float freq = LORAWAN_US915_SB2[chIdx];

            heliumConfigRadio(freq, sf);
            fillRandomPayload(25);
            _radio->transmit(_payload, 25);
            _packetCount++;

            _lastFreq = freq;
            _lastSF = sf;
            _hotspots[i].lastChIdx = chIdx;
            _hotspots[i].nextTxMs = now + rngRange(25000, 35000);  // ~30s per hotspot

            Serial.printf("  Hotspot %u: SF%u @ %.1f MHz\n", i + 1, sf, freq);
        }
    }
}

// ============================================================
// MODE 3: LoRaWAN EU868 — v2 ref §4.2
// ============================================================
// 3 mandatory channels, random SF, 60s interval

static unsigned long _euNextTxMs = 0;

static void euStart() {
    _euNextTxMs = millis() + 3000;  // first TX after 3s

    Serial.printf("[LoRaWAN-EU868] 3ch 868.1/868.3/868.5MHz SF7-SF12 BW125\n");
    Serial.printf("  Sync: 0x%02X (public)  Preamble: %u sym  Interval: ~60s\n",
                  SYNC_WORD_LORAWAN, LORAWAN_PREAMBLE_LEN);
    Serial.println("  NOTE: ETSI 1% duty cycle applies in real deployments");
    Serial.println("  SENTRY-RF should: CLEAR (sporadic, low diversity)");
}

static void euUpdate() {
    unsigned long now = millis();
    if (now < _euNextTxMs) return;

    // Random mandatory channel
    uint8_t chIdx = rng() % LORAWAN_EU868_CH_COUNT;
    float freq = LORAWAN_EU868_CHANNELS[chIdx];
    uint8_t sf = (uint8_t)rngRange(7, 12);

    _radio->standby();
    _radio->begin(
        freq, 125.0, sf, 5,
        SYNC_WORD_LORAWAN,
        rfGetPower(),
        LORAWAN_PREAMBLE_LEN,
        1.8, false
    );
    _radio->explicitHeader();

    fillRandomPayload(rngRange(15, 40));
    _radio->transmit(_payload, rngRange(15, 40));
    _packetCount++;

    _lastFreq = freq;
    _lastSF = sf;
    _euNextTxMs = now + rngRange(50000, 70000);  // ~60s

    Serial.printf("  EU868 TX: SF%u @ %.1f MHz\n", sf, freq);
}

// ============================================================
// Public API
// ============================================================

void infraInit(SX1262 *radio) {
    _radio = radio;
    _running = false;
}

void infraStart(InfraMode mode) {
    if (!_radio) return;

    _mode = mode;
    _packetCount = 0;
    _lastFreq = 0;
    _lastSF = 7;
    _startMs = millis();
    _rng = (uint32_t)(micros() ^ 0xBEEF);

    _radio->reset();
    delay(100);

    switch (mode) {
    case INFRA_MESHTASTIC: meshStart(); break;
    case INFRA_HELIUM_POC: heliumStart(); break;
    case INFRA_LORAWAN_EU: euStart(); break;
    }

    _running = true;
}

void infraStop() {
    if (!_radio) return;
    _radio->standby();
    _running = false;

    // Restore radio to default
    _radio->begin(915.0, 125.0, 9, 7, SYNC_WORD_ELRS, 10, 8, 1.8, false);

    const char* names[] = { "Meshtastic", "Helium PoC", "LoRaWAN EU868" };
    Serial.printf("[%s] Stopped: %lu packets, %lu sec\n",
                  names[_mode], (unsigned long)_packetCount,
                  (unsigned long)((millis() - _startMs) / 1000));
}

void infraUpdate() {
    if (!_running || !_radio) return;

    switch (_mode) {
    case INFRA_MESHTASTIC: meshUpdate(); break;
    case INFRA_HELIUM_POC: heliumUpdate(); break;
    case INFRA_LORAWAN_EU: euUpdate(); break;
    }
}

InfraParams infraGetParams() {
    return InfraParams{
        _mode,
        _lastFreq,
        _lastSF,
        _packetCount,
        _running ? (uint32_t)((millis() - _startMs) / 1000) : 0,
        rfGetPower(),
        _running,
    };
}
