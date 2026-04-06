#ifndef CUSTOM_LORA_H
#define CUSTOM_LORA_H

#include <RadioLib.h>

// ============================================================
// Custom LoRa Direct — User-Configurable LoRa Transmitter
// v2 ref §3.6 — Simulates non-standard drone LoRa links
// ============================================================

enum CustomHopMode {
    HOP_NONE = 0,     // Fixed single frequency
    HOP_2CH  = 2,     // 2-channel alternating (±500 kHz)
    HOP_RANDOM = 255, // N-channel random (N stored separately)
};

struct CustomLoraConfig {
    float    freqMHz;       // Center frequency (default 915.0)
    uint8_t  sf;            // Spreading factor 5-12 (default 7)
    float    bwKHz;         // Bandwidth in kHz (default 125)
    uint8_t  rateHz;        // Packet rate 1-100 (default 10)
    int8_t   powerDbm;      // TX power (default 10)
    uint8_t  syncWord;      // LoRa sync word (default 0x12)
    uint8_t  hopMode;       // 0=none, 2=alternating, N=random N-ch
    uint8_t  hopChannels;   // Number of channels for random hop mode
};

struct CustomLoraParams {
    float    currentMHz;
    uint32_t packetCount;
    uint32_t hopCount;
    CustomLoraConfig cfg;
    bool     running;
};

void customLoraInit(SX1262 *radio);
void customLoraConfigure(const char *cmdStr);  // parse "f915.5", "s7", "b125", etc.
void customLoraPrintConfig();                  // print current settings
void customLoraStart();
void customLoraStop();
void customLoraUpdate();        // call every loop()
CustomLoraParams customLoraGetParams();

#endif // CUSTOM_LORA_H
