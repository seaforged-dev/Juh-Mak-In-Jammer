#include <Arduino.h>
#include "board_config.h"
#include "menu.h"
#include "rf_modes.h"
#include "false_positive.h"
#include "rid_spoofer.h"
#include "combined_mode.h"
#include "swarm_sim.h"
#include "crossfire.h"
#include "power_ramp.h"
#include "sik_radio.h"
#include "mlrs_sim.h"
#include "custom_lora.h"

// ============================================================
// Menu state machine + button debouncer
// ============================================================

static Adafruit_SSD1306 *_oled = nullptr;
static AppState _state = STATE_MAIN_MENU;
static uint8_t _mainSel = 0;
static uint8_t _siggenSel = 0;
static uint8_t _fpSel = 0;
static bool _needsRedraw = true;

// --- Button debounce state ---
static constexpr unsigned long DEBOUNCE_MS   = 40;
static constexpr unsigned long LONG_PRESS_MS = 400;

static bool     _lastRaw      = HIGH;   // idle = HIGH (pull-up)
static bool     _stable       = HIGH;
static unsigned long _lastChange = 0;
static unsigned long _pressStart = 0;
static bool     _pressed      = false;

// ============================================================
// Button reader — non-blocking debounce + short/long detection
// ============================================================
ButtonPress buttonRead() {
    bool raw = digitalRead(BOOT_BTN);
    unsigned long now = millis();

    // Debounce: only accept a new level after it's stable for DEBOUNCE_MS
    if (raw != _lastRaw) {
        _lastChange = now;
        _lastRaw = raw;
    }

    if ((now - _lastChange) >= DEBOUNCE_MS && raw != _stable) {
        _stable = raw;

        if (_stable == LOW) {
            // Button just pressed down
            _pressStart = now;
            _pressed = true;
        } else if (_pressed) {
            // Button just released — classify the press
            _pressed = false;
            unsigned long duration = now - _pressStart;
            ButtonPress result = (duration >= LONG_PRESS_MS) ? BTN_LONG : BTN_SHORT;
            Serial.printf("BTN: %s (%lu ms)\n",
                          result == BTN_LONG ? "LONG" : "SHORT", duration);
            return result;
        }
    }
    return BTN_NONE;
}

// ============================================================
// OLED drawing helpers
// ============================================================

static void drawHeader(const char *title) {
    _oled->setTextSize(1);
    _oled->setTextColor(SSD1306_WHITE);
    _oled->setCursor(0, 0);
    _oled->println(title);
    // Thin separator line under header
    _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
}

static void drawMenuItem(uint8_t index, uint8_t selected, const char *label, uint8_t y) {
    _oled->setCursor(0, y);
    _oled->print(index == selected ? "> " : "  ");
    _oled->print(label);
}

// ============================================================
// Screen renderers
// ============================================================

static void drawMainMenu() {
    _oled->clearDisplay();
    drawHeader("JUH-MAK-IN JAMMER");

    static const char *labels[MAIN_COUNT] = {
        "[1] RID Spoofer",
        "[2] RF Sig Gen",
        "[3] False Pos Gen",
        "[4] Combined",
        "[5] Swarm Sim",
    };

    // Scrollable: show 4 items at a time
    static const uint8_t VISIBLE = 4;
    uint8_t scrollTop = 0;
    if (_mainSel >= VISIBLE) scrollTop = _mainSel - VISIBLE + 1;

    for (uint8_t v = 0; v < VISIBLE && (scrollTop + v) < MAIN_COUNT; v++) {
        uint8_t i = scrollTop + v;
        drawMenuItem(i, _mainSel, labels[i], 14 + v * 10);
    }

    // Scroll indicator
    if (scrollTop + VISIBLE < MAIN_COUNT) {
        _oled->setCursor(122, 44);
        _oled->print("v");
    }

    _oled->setCursor(0, 56);
    _oled->print("SHORT=nav  LONG=sel");
    _oled->display();
}

static void drawSigGenMenu() {
    _oled->clearDisplay();
    drawHeader("RF Signal Generator");

    static const char *labels[SIGGEN_COUNT] = {
        "CW Tone",
        "Band Sweep",
        "ELRS 915 FHSS",
        "Crossfire 915",
        "Power Ramp",
        "<< Back",
    };

    static const uint8_t VISIBLE = 4;
    uint8_t scrollTop = 0;
    if (_siggenSel >= VISIBLE) scrollTop = _siggenSel - VISIBLE + 1;

    for (uint8_t v = 0; v < VISIBLE && (scrollTop + v) < SIGGEN_COUNT; v++) {
        uint8_t i = scrollTop + v;
        drawMenuItem(i, _siggenSel, labels[i], 14 + v * 10);
    }

    if (scrollTop + VISIBLE < SIGGEN_COUNT) {
        _oled->setCursor(122, 44);
        _oled->print("v");
    }

    _oled->setCursor(0, 56);
    _oled->print("SHORT=nav  LONG=sel");
    _oled->display();
}

static void drawCwActive() {
    _oled->clearDisplay();
    drawHeader("CW Tone - TX");

    CwParams p = cwGetParams();

    _oled->setCursor(0, 14);
    _oled->printf("Freq: %.2f MHz", p.freqMHz);
    _oled->setCursor(0, 26);
    _oled->printf("Power: %d dBm", p.powerDbm);
    _oled->setCursor(0, 38);
    _oled->print("Status: ");
    _oled->print(p.transmitting ? "TX ON" : "TX OFF");

    _oled->setCursor(0, 52);
    _oled->print("SHORT=freq  LONG=stop");
    _oled->display();
}

static void drawSweepActive() {
    _oled->clearDisplay();
    drawHeader("Band Sweep - TX");

    SweepParams s = sweepGetParams();

    _oled->setCursor(0, 14);
    _oled->printf("%.1f -> %.1f MHz", s.startMHz, s.endMHz);
    _oled->setCursor(0, 24);
    _oled->printf("Now: %.2f MHz", s.currentMHz);

    // Progress bar: visual indicator of sweep position
    uint8_t barWidth = 100;
    uint8_t barX = 14;
    uint8_t barY = 34;
    _oled->drawRect(barX, barY, barWidth, 6, SSD1306_WHITE);
    if (s.totalSteps > 0) {
        uint8_t fill = (uint8_t)((uint32_t)s.stepIndex * (barWidth - 2) / s.totalSteps);
        _oled->fillRect(barX + 1, barY + 1, fill, 4, SSD1306_WHITE);
    }

    _oled->setCursor(0, 42);
    _oled->printf("Step:%.0fkHz Dw:%uus", s.stepMHz * 1000.0f, s.dwellUs);
    _oled->setCursor(0, 52);
    _oled->printf("Pwr:%ddBm %s", s.powerDbm, s.running ? "TX" : "OFF");

    // No room for help text — sweep shows enough info
    _oled->display();
}

static void drawElrsActive() {
    _oled->clearDisplay();
    drawHeader("ELRS 915 - FHSS TX");

    ElrsParams e = elrsGetParams();

    _oled->setCursor(0, 14);
    _oled->printf("Ch:%u/40 %.1f MHz", e.channelIndex, e.currentMHz);
    _oled->setCursor(0, 24);
    _oled->printf("Pkts: %lu", (unsigned long)e.packetCount);
    _oled->setCursor(0, 34);
    _oled->printf("Hops: %lu", (unsigned long)e.hopCount);
    _oled->setCursor(0, 44);
    _oled->printf("Pwr:%ddBm SF6 BW500", e.powerDbm);

    _oled->setCursor(0, 56);
    _oled->print("LONG=stop");
    _oled->display();
}

static void drawFpMenu() {
    _oled->clearDisplay();
    drawHeader("False Positive Gen");

    static const char *labels[FP_COUNT] = {
        "LoRaWAN Sim",
        "ISM Burst Noise",
        "Mixed (IoT+ELRS)",
        "<< Back",
    };

    for (uint8_t i = 0; i < FP_COUNT; i++) {
        drawMenuItem(i, _fpSel, labels[i], 14 + i * 12);
    }

    _oled->display();
}

static void drawFpActive() {
    _oled->clearDisplay();

    FpParams f = fpGetParams();

    // Header varies by sub-mode
    switch (f.mode) {
    case FP_LORAWAN:
        drawHeader("FP: LoRaWAN");
        _oled->setCursor(0, 14);
        _oled->printf("Freq: %.1f MHz", f.lastFreqMHz);
        _oled->setCursor(0, 24);
        _oled->printf("Last SF: %u  BW: 125k", f.lastSF);
        _oled->setCursor(0, 34);
        _oled->printf("Packets: %lu", (unsigned long)f.loraPacketCount);
        _oled->setCursor(0, 44);
        _oled->printf("Pwr: %d dBm", f.powerDbm);
        break;

    case FP_ISM_BURST:
        drawHeader("FP: ISM Bursts");
        _oled->setCursor(0, 14);
        _oled->printf("Last: %.2f MHz", f.lastFreqMHz);
        _oled->setCursor(0, 24);
        _oled->print("Type: Random FSK");
        _oled->setCursor(0, 34);
        _oled->printf("Bursts: %lu", (unsigned long)f.burstCount);
        _oled->setCursor(0, 44);
        _oled->printf("Pwr: %d dBm", f.powerDbm);
        break;

    case FP_MIXED:
        drawHeader("FP: Mixed IoT+ELRS");
        _oled->setCursor(0, 14);
        _oled->printf("ELRS pkts: %lu", (unsigned long)f.elrsPacketCount);
        _oled->setCursor(0, 24);
        _oled->printf("LoRaWAN:   %lu", (unsigned long)f.loraPacketCount);
        _oled->setCursor(0, 34);
        _oled->printf("Freq: %.2f MHz", f.lastFreqMHz);
        _oled->setCursor(0, 44);
        _oled->printf("Pwr: %d dBm", f.powerDbm);
        break;

    default:
        break;
    }

    _oled->setCursor(0, 56);
    _oled->print("LONG=stop");
    _oled->display();
}

static void drawRidActive() {
    _oled->clearDisplay();
    drawHeader("RID Spoofer - TX");

    RidParams r = ridGetParams();

    _oled->setCursor(0, 14);
    _oled->printf("%.4f, %.4f", r.latitude, r.longitude);
    _oled->setCursor(0, 24);
    _oled->printf("Alt: %.0f m", r.altitude);
    _oled->setCursor(0, 34);
    _oled->printf("W:%lu B:%lu",
                  (unsigned long)r.wifiPackets,
                  (unsigned long)r.blePackets);
    _oled->setCursor(0, 44);
    _oled->print("TX: WiFi + BLE");

    _oled->setCursor(0, 56);
    _oled->print("LONG=stop");
    _oled->display();
}

static void drawCombinedActive() {
    _oled->clearDisplay();
    drawHeader("COMBINED - DUAL TX");

    CombinedParams c = combinedGetParams();

    _oled->setCursor(0, 14);
    _oled->printf("RID: W:%lu B:%lu",
                  (unsigned long)c.ridWifiPkts,
                  (unsigned long)c.ridBlePkts);
    _oled->setCursor(0, 24);
    _oled->printf("ELRS: %lu pk %lu hp",
                  (unsigned long)c.elrsPkts,
                  (unsigned long)c.elrsHops);
    _oled->setCursor(0, 34);
    _oled->print("C0:WiFi+BLE C1:ELRS");
    _oled->setCursor(0, 44);
    _oled->printf("Elapsed: %lu sec", (unsigned long)c.elapsedSec);

    _oled->setCursor(0, 56);
    _oled->print("LONG=stop");
    _oled->display();
}

// ============================================================
// State machine
// ============================================================

void menuInit(Adafruit_SSD1306 *oled) {
    _oled = oled;
    _state = STATE_MAIN_MENU;
    _mainSel = MAIN_RF_SIGGEN;  // default to first working mode
    _siggenSel = 0;
    _needsRedraw = true;
}

AppState menuGetState() {
    return _state;
}

void menuSetState(AppState s) {
    _state = s;
    _needsRedraw = true;
}

void menuRequestRedraw() {
    _needsRedraw = true;
}

void menuUpdate() {
    ButtonPress btn = buttonRead();

    switch (_state) {

    // --- Main Menu ---
    case STATE_MAIN_MENU:
        if (btn == BTN_SHORT) {
            _mainSel = (_mainSel + 1) % MAIN_COUNT;
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            if (_mainSel == MAIN_RF_SIGGEN) {
                _state = STATE_SIGGEN_MENU;
                _siggenSel = 0;
                _needsRedraw = true;
            } else if (_mainSel == MAIN_FALSE_POS) {
                _state = STATE_FALSEPOS_MENU;
                _fpSel = 0;
                _needsRedraw = true;
            } else if (_mainSel == MAIN_RID_SPOOFER) {
                ridStart();
                _state = STATE_RID_ACTIVE;
                _needsRedraw = true;
            } else if (_mainSel == MAIN_COMBINED) {
                combinedStart();
                _state = STATE_COMBINED_ACTIVE;
                _needsRedraw = true;
            } else if (_mainSel == MAIN_SWARM) {
                swarmStart();
                _state = STATE_SWARM_ACTIVE;
                _needsRedraw = true;
            }
        }
        if (_needsRedraw) { drawMainMenu(); _needsRedraw = false; }
        break;

    // --- Signal Generator Submenu ---
    case STATE_SIGGEN_MENU:
        if (btn == BTN_SHORT) {
            _siggenSel = (_siggenSel + 1) % SIGGEN_COUNT;
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            if (_siggenSel == SIGGEN_CW_TONE) {
                cwStart();
                _state = STATE_CW_ACTIVE;
                _needsRedraw = true;
            } else if (_siggenSel == SIGGEN_SWEEP) {
                sweepStart();
                _state = STATE_SWEEP_ACTIVE;
                _needsRedraw = true;
            } else if (_siggenSel == SIGGEN_ELRS) {
                elrsStart();
                _state = STATE_ELRS_ACTIVE;
                _needsRedraw = true;
            } else if (_siggenSel == SIGGEN_CROSSFIRE) {
                crossfireStart();
                _state = STATE_CROSSFIRE_ACTIVE;
                _needsRedraw = true;
            } else if (_siggenSel == SIGGEN_POWER_RAMP) {
                powerRampStart();
                _state = STATE_RAMP_ACTIVE;
                _needsRedraw = true;
            } else if (_siggenSel == SIGGEN_BACK) {
                _state = STATE_MAIN_MENU;
                _needsRedraw = true;
            }
        }
        if (_needsRedraw) { drawSigGenMenu(); _needsRedraw = false; }
        break;

    // --- CW Tone Active ---
    case STATE_CW_ACTIVE:
        if (btn == BTN_SHORT) {
            cwCycleFreq();
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            cwStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        // Refresh display every 500 ms to show live status
        {
            static unsigned long lastRefresh = 0;
            if (_needsRedraw || (millis() - lastRefresh > 500)) {
                drawCwActive();
                lastRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Frequency Sweep Active ---
    case STATE_SWEEP_ACTIVE:
        // Advance the sweep each loop iteration (non-blocking, dwell-gated)
        sweepUpdate();

        if (btn == BTN_SHORT) {
            sweepCycleStep();
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            sweepStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        // Refresh display at ~10 Hz so the frequency counter feels live
        {
            static unsigned long lastSweepRefresh = 0;
            if (_needsRedraw || (millis() - lastSweepRefresh > 100)) {
                drawSweepActive();
                lastSweepRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- ELRS FHSS Active ---
    case STATE_ELRS_ACTIVE:
        elrsUpdate();

        if (btn == BTN_LONG) {
            elrsStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        // Refresh at 4 Hz — hop rate (200 Hz) is too fast to show every hop
        {
            static unsigned long lastElrsRefresh = 0;
            if (_needsRedraw || (millis() - lastElrsRefresh > 250)) {
                drawElrsActive();
                lastElrsRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- False Positive Submenu ---
    case STATE_FALSEPOS_MENU:
        if (btn == BTN_SHORT) {
            _fpSel = (_fpSel + 1) % FP_COUNT;
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            if (_fpSel == FP_BACK) {
                _state = STATE_MAIN_MENU;
                _needsRedraw = true;
            } else {
                fpStart((FpMode)_fpSel);
                _state = STATE_FP_ACTIVE;
                _needsRedraw = true;
            }
        }
        if (_needsRedraw) { drawFpMenu(); _needsRedraw = false; }
        break;

    // --- False Positive Active ---
    case STATE_FP_ACTIVE:
        fpUpdate();

        if (btn == BTN_LONG) {
            fpStop();
            _state = STATE_FALSEPOS_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastFpRefresh = 0;
            if (_needsRedraw || (millis() - lastFpRefresh > 500)) {
                drawFpActive();
                lastFpRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- RID Spoofer Active ---
    case STATE_RID_ACTIVE:
        ridUpdate();

        if (btn == BTN_LONG) {
            ridStop();
            _state = STATE_MAIN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastRidRefresh = 0;
            if (_needsRedraw || (millis() - lastRidRefresh > 500)) {
                drawRidActive();
                lastRidRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Combined Mode Active ---
    case STATE_COMBINED_ACTIVE:
        combinedUpdate();

        if (btn == BTN_LONG) {
            combinedStop();
            _state = STATE_MAIN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastCmbRefresh = 0;
            if (_needsRedraw || (millis() - lastCmbRefresh > 500)) {
                drawCombinedActive();
                lastCmbRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Crossfire Active ---
    case STATE_CROSSFIRE_ACTIVE:
        crossfireUpdate();

        if (btn == BTN_LONG) {
            crossfireStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastCrsfRefresh = 0;
            if (_needsRedraw || (millis() - lastCrsfRefresh > 250)) {
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("CROSSFIRE 915 - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                CrossfireParams cp = crossfireGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("Ch: %u/100  %.1f MHz", cp.channelIndex, cp.currentMHz);
                _oled->setCursor(0, 24);
                _oled->printf("Pkts: %lu", (unsigned long)cp.packetCount);
                _oled->setCursor(0, 34);
                _oled->printf("Hops: %lu", (unsigned long)cp.hopCount);
                _oled->setCursor(0, 44);
                _oled->printf("Pwr:%ddBm FSK 85k", cp.powerDbm);
                _oled->setCursor(0, 56);
                _oled->print("LONG=stop");
                _oled->display();

                lastCrsfRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Power Ramp Active ---
    case STATE_RAMP_ACTIVE:
        powerRampUpdate();

        if (btn == BTN_SHORT) {
            powerRampCycleDuration();
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            powerRampStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastRampRefresh = 0;
            if (_needsRedraw || (millis() - lastRampRefresh > 250)) {
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("POWER RAMP - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                PowerRampParams rp = powerRampGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("Pwr: %+d dBm  %s", rp.currentPowerDbm,
                              rp.ascending ? ">>>" : "<<<");
                // Power bar
                uint8_t barFill = (uint8_t)(((float)(rp.currentPowerDbm + 9) / 31.0f) * 100);
                _oled->drawRect(14, 25, 100, 6, SSD1306_WHITE);
                _oled->fillRect(15, 26, barFill, 4, SSD1306_WHITE);

                _oled->setCursor(0, 34);
                _oled->printf("T: %lu / %lu sec", (unsigned long)rp.elapsedSec,
                              (unsigned long)rp.rampDurationSec);
                _oled->setCursor(0, 44);
                _oled->printf("Pkts: %lu  %.1f MHz", (unsigned long)rp.packetCount, rp.currentMHz);
                _oled->setCursor(0, 56);
                _oled->print("SH=dur  LG=stop");
                _oled->display();

                lastRampRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Swarm Simulator Active ---
    case STATE_SWARM_ACTIVE:
        swarmUpdate();

        if (btn == BTN_SHORT) {
            swarmCycleCount();
            _needsRedraw = true;
        } else if (btn == BTN_LONG) {
            swarmStop();
            _state = STATE_MAIN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastSwarmRefresh = 0;
            if (_needsRedraw || (millis() - lastSwarmRefresh > 500)) {
                // Draw swarm status on OLED
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("SWARM SIM - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                SwarmParams sp = swarmGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("Drones: %d", sp.droneCount);
                _oled->setCursor(0, 24);
                _oled->printf("Beacons: %lu", (unsigned long)sp.beaconCount);
                _oled->setCursor(0, 34);
                _oled->printf("Elapsed: %lu sec", (unsigned long)sp.elapsedSec);
                _oled->setCursor(0, 56);
                _oled->print("SH=count  LG=stop");
                _oled->display();

                lastSwarmRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- SiK Radio Active ---
    case STATE_SIK_ACTIVE:
        sikUpdate();

        if (btn == BTN_LONG) {
            sikStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastSikRefresh = 0;
            if (_needsRedraw || (millis() - lastSikRefresh > 250)) {
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("SiK RADIO - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                SikParams sp = sikGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("Ch:%u/50  %.1f MHz", sp.channelIndex, sp.currentMHz);
                _oled->setCursor(0, 24);
                _oled->printf("GFSK %.0f kbps", sp.airSpeedKbps);
                _oled->setCursor(0, 34);
                _oled->printf("Pkts: %lu  Hops: %lu", (unsigned long)sp.packetCount, (unsigned long)sp.hopCount);
                _oled->setCursor(0, 44);
                _oled->printf("Pwr: %d dBm", sp.powerDbm);
                _oled->setCursor(0, 56);
                _oled->print("LONG=stop");
                _oled->display();

                lastSikRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- mLRS Active ---
    case STATE_MLRS_ACTIVE:
        mlrsUpdate();

        if (btn == BTN_LONG) {
            mlrsStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastMlrsRefresh = 0;
            if (_needsRedraw || (millis() - lastMlrsRefresh > 250)) {
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("mLRS - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                MlrsParams mp = mlrsGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("Ch:%u/20  %.1f MHz", mp.channelIndex, mp.currentMHz);
                _oled->setCursor(0, 24);
                _oled->printf("%s  %uHz sym", mp.isLoRa ? "LoRa" : "FSK", mp.rateHz);
                _oled->setCursor(0, 34);
                _oled->printf("Pkts: %lu  Hops: %lu", (unsigned long)mp.packetCount, (unsigned long)mp.hopCount);
                _oled->setCursor(0, 44);
                _oled->printf("Pwr: %d dBm", mp.powerDbm);
                _oled->setCursor(0, 56);
                _oled->print("LONG=stop");
                _oled->display();

                lastMlrsRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;

    // --- Custom LoRa Active ---
    case STATE_CUSTOM_LORA_ACTIVE:
        customLoraUpdate();

        if (btn == BTN_LONG) {
            customLoraStop();
            _state = STATE_SIGGEN_MENU;
            _needsRedraw = true;
        }
        {
            static unsigned long lastClRefresh = 0;
            if (_needsRedraw || (millis() - lastClRefresh > 250)) {
                _oled->clearDisplay();
                _oled->setTextSize(1);
                _oled->setTextColor(SSD1306_WHITE);
                _oled->setCursor(0, 0);
                _oled->println("CUSTOM LORA - TX");
                _oled->drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

                CustomLoraParams cp = customLoraGetParams();
                _oled->setCursor(0, 14);
                _oled->printf("%.3f MHz", cp.currentMHz);
                _oled->setCursor(0, 24);
                _oled->printf("SF%u BW%.0f %uHz", cp.cfg.sf, cp.cfg.bwKHz, cp.cfg.rateHz);
                _oled->setCursor(0, 34);
                _oled->printf("Pkts:%lu Hops:%lu", (unsigned long)cp.packetCount, (unsigned long)cp.hopCount);
                _oled->setCursor(0, 44);
                _oled->printf("Sync:0x%02X Pwr:%ddBm", cp.cfg.syncWord, cp.cfg.powerDbm);
                _oled->setCursor(0, 56);
                _oled->print("LONG=stop");
                _oled->display();

                lastClRefresh = millis();
                _needsRedraw = false;
            }
        }
        break;
    }
}
