#include <Arduino.h>
#include "custom_lora.h"
#include "protocol_params.h"

// ============================================================
// Custom LoRa Direct — v2 ref §3.6
// ============================================================
// User-configurable LoRa transmitter for simulating non-standard
// drone links that don't match any known protocol signature.

static SX1262 *_radio = nullptr;

// Default configuration
static CustomLoraConfig _cfg = {
    .freqMHz     = 915.0f,
    .sf          = 7,
    .bwKHz       = 125.0f,
    .rateHz      = 10,
    .powerDbm    = 10,
    .syncWord    = SYNC_WORD_ELRS,  // 0x12
    .hopMode     = HOP_NONE,
    .hopChannels = 0,
};

// Hop channel table (for 2-ch and N-ch modes)
static const uint8_t MAX_HOP_CH = 16;
static float _hopFreqs[MAX_HOP_CH];
static uint8_t _numHopCh = 0;
static uint8_t _hopIdx = 0;

// State
static bool     _running     = false;
static uint32_t _packetCount = 0;
static uint32_t _hopCount    = 0;
static float    _currentMHz  = 0;
static unsigned long _lastTxUs = 0;

// Dummy payload
static const uint8_t CUSTOM_PAYLOAD[] = {
    0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF, 0x55, 0xAA
};

// Build hop channel table based on config
static void buildHopTable() {
    if (_cfg.hopMode == HOP_NONE) {
        _numHopCh = 1;
        _hopFreqs[0] = _cfg.freqMHz;
    } else if (_cfg.hopMode == HOP_2CH) {
        _numHopCh = 2;
        _hopFreqs[0] = _cfg.freqMHz - 0.5f;  // center - 500 kHz
        _hopFreqs[1] = _cfg.freqMHz + 0.5f;  // center + 500 kHz
    } else {
        // Random N-channel: spread across ±2 MHz from center
        uint8_t n = _cfg.hopChannels;
        if (n < 2) n = 2;
        if (n > MAX_HOP_CH) n = MAX_HOP_CH;
        _numHopCh = n;
        float span = 4.0f;  // ±2 MHz = 4 MHz total
        for (uint8_t i = 0; i < n; i++) {
            _hopFreqs[i] = (_cfg.freqMHz - 2.0f) + (i * span / (n - 1));
        }
    }
}

// ============================================================
// Public API
// ============================================================

void customLoraInit(SX1262 *radio) {
    _radio = radio;
    _running = false;
}

void customLoraConfigure(const char *cmd) {
    if (!cmd || !cmd[0]) return;

    char sub = cmd[0];
    const char *val = &cmd[1];

    switch (sub) {
    case 'f': {  // frequency: uf915.5
        float f = atof(val);
        if (f >= 860.0f && f <= 930.0f) {
            _cfg.freqMHz = f;
            Serial.printf("[CustomLoRa] Freq: %.3f MHz\n", _cfg.freqMHz);
        } else {
            Serial.println("[CustomLoRa] Freq must be 860-930 MHz");
        }
        break;
    }
    case 's': {  // SF: us7
        uint8_t s = atoi(val);
        if (s >= 5 && s <= 12) {
            _cfg.sf = s;
            Serial.printf("[CustomLoRa] SF: %u\n", _cfg.sf);
        }
        break;
    }
    case 'b': {  // BW: ub125
        float b = atof(val);
        // Accept common BW values
        if (b >= 7.0f && b <= 500.0f) {
            _cfg.bwKHz = b;
            Serial.printf("[CustomLoRa] BW: %.1f kHz\n", _cfg.bwKHz);
        }
        break;
    }
    case 'r': {  // rate: ur20
        uint8_t r = atoi(val);
        if (r >= 1 && r <= 100) {
            _cfg.rateHz = r;
            Serial.printf("[CustomLoRa] Rate: %u Hz\n", _cfg.rateHz);
        }
        break;
    }
    case 'h': {  // hop mode: uh0, uh2, uh5
        uint8_t h = atoi(val);
        if (h == 0) {
            _cfg.hopMode = HOP_NONE;
            _cfg.hopChannels = 0;
            Serial.println("[CustomLoRa] Hop: none (fixed freq)");
        } else if (h == 2) {
            _cfg.hopMode = HOP_2CH;
            _cfg.hopChannels = 2;
            Serial.println("[CustomLoRa] Hop: 2-channel alternating");
        } else if (h >= 3 && h <= MAX_HOP_CH) {
            _cfg.hopMode = HOP_RANDOM;
            _cfg.hopChannels = h;
            Serial.printf("[CustomLoRa] Hop: %u-channel random\n", h);
        }
        break;
    }
    case 'p': {  // power: up10
        int8_t p = atoi(val);
        if (p >= -9 && p <= 22) {
            _cfg.powerDbm = p;
            Serial.printf("[CustomLoRa] Power: %d dBm\n", _cfg.powerDbm);
        }
        break;
    }
    case 'w': {  // sync word: uw12, uw34, uw2B
        uint8_t w = (uint8_t)strtol(val, NULL, 16);
        if (w > 0) {
            _cfg.syncWord = w;
            Serial.printf("[CustomLoRa] SyncWord: 0x%02X\n", _cfg.syncWord);
        }
        break;
    }
    default:
        Serial.printf("[CustomLoRa] Unknown param '%c'\n", sub);
        break;
    }
}

void customLoraPrintConfig() {
    Serial.println("[CustomLoRa] Current settings:");
    Serial.printf("  Freq: %.3f MHz  SF%u  BW%.1fkHz\n", _cfg.freqMHz, _cfg.sf, _cfg.bwKHz);
    Serial.printf("  Rate: %u Hz  Power: %d dBm  SyncWord: 0x%02X\n",
                  _cfg.rateHz, _cfg.powerDbm, _cfg.syncWord);
    if (_cfg.hopMode == HOP_NONE) {
        Serial.println("  Hop: none (fixed frequency)");
    } else if (_cfg.hopMode == HOP_2CH) {
        Serial.printf("  Hop: 2-channel alternating (%.1f / %.1f MHz)\n",
                      _cfg.freqMHz - 0.5f, _cfg.freqMHz + 0.5f);
    } else {
        Serial.printf("  Hop: %u-channel random (%.1f-%.1f MHz)\n",
                      _cfg.hopChannels, _cfg.freqMHz - 2.0f, _cfg.freqMHz + 2.0f);
    }
}

void customLoraStart() {
    if (!_radio) return;

    buildHopTable();
    _hopIdx = 0;
    _packetCount = 0;
    _hopCount = 0;

    _radio->reset();
    delay(100);

    int state = _radio->begin(
        _hopFreqs[0],
        _cfg.bwKHz,
        _cfg.sf,
        7,                  // CR 4/7
        _cfg.syncWord,
        _cfg.powerDbm,
        6,                  // preamble
        1.8, false
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CustomLoRa] radio config FAILED (error %d)\n", state);
        _running = false;
        return;
    }

    _radio->explicitHeader();
    _radio->setCurrentLimit(140.0);

    _currentMHz = _hopFreqs[0];
    _running = true;
    _lastTxUs = micros();

    _radio->startTransmit(CUSTOM_PAYLOAD, sizeof(CUSTOM_PAYLOAD));
    _packetCount++;

    // Protocol info output
    Serial.printf("[CustomLoRa] %.3fMHz SF%u/BW%.0f %uHz sync=0x%02X %ddBm",
                  _cfg.freqMHz, _cfg.sf, _cfg.bwKHz, _cfg.rateHz,
                  _cfg.syncWord, _cfg.powerDbm);
    if (_cfg.hopMode == HOP_NONE) {
        Serial.println(" fixed-freq");
    } else if (_cfg.hopMode == HOP_2CH) {
        Serial.println(" 2ch-alt");
    } else {
        Serial.printf(" %uch-random\n", _cfg.hopChannels);
    }
}

void customLoraStop() {
    if (!_radio) return;
    _radio->standby();
    _running = false;
    Serial.printf("[CustomLoRa] TX OFF: %lu packets, %lu hops\n",
                  (unsigned long)_packetCount, (unsigned long)_hopCount);
}

void customLoraUpdate() {
    if (!_running || !_radio) return;

    uint32_t intervalUs = 1000000UL / _cfg.rateHz;
    unsigned long nowUs = micros();
    if ((nowUs - _lastTxUs) < intervalUs) return;
    _lastTxUs += intervalUs;

    // Hop if multi-channel
    if (_numHopCh > 1) {
        if (_cfg.hopMode == HOP_2CH) {
            _hopIdx = (_hopIdx + 1) % 2;
        } else {
            _hopIdx = esp_random() % _numHopCh;  // random channel selection
        }
        _hopCount++;

        float nextFreq = _hopFreqs[_hopIdx];
        if (nextFreq != _currentMHz) {
            _currentMHz = nextFreq;
            _radio->standby();
            _radio->setFrequency(nextFreq);
        }
    }

    _radio->startTransmit(CUSTOM_PAYLOAD, sizeof(CUSTOM_PAYLOAD));
    _packetCount++;
}

CustomLoraParams customLoraGetParams() {
    return CustomLoraParams{
        _currentMHz,
        _packetCount,
        _hopCount,
        _cfg,
        _running,
    };
}
