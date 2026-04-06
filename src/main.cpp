#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include "board_config.h"
#include "version.h"
#include "menu.h"
#include "rf_modes.h"
#include "false_positive.h"
#include "rid_spoofer.h"
#include "combined_mode.h"
#include "swarm_sim.h"
#include "crossfire.h"
#include "power_ramp.h"
#include "protocol_params.h"
#include "splash.h"

// --- OLED Display ---
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// --- LoRa Radio ---
SPIClass loraSPI(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// --- Boot Splash Screen (shown for 2 seconds) ---
static void showBootScreen() {
    display.clearDisplay();

    // Drone + RF wave arcs bitmap in the top 40 pixels
    display.drawBitmap(0, 0, splash_bmp, SPLASH_WIDTH, SPLASH_HEIGHT, SSD1306_WHITE);

    // Project name centered at y=44
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 44);
    display.print("JUH-MAK-IN JAMMER");

    // Version string centered at y=56
    display.setCursor(40, 56);
    display.printf("v%s", JAMMER_VERSION);

    display.display();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.printf("  %s v%s\n", JAMMER_NAME, JAMMER_VERSION);
    Serial.printf("  Component: %s\n", JAMMER_COMPONENT);
    Serial.printf("  Board: %s\n", BOARD_NAME);
    Serial.println("================================");

    // --- LED ---
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // --- Boot Button ---
    pinMode(BOOT_BTN, INPUT_PULLUP);

    // --- OLED Init ---
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("ERROR: OLED init failed!");
    } else {
        Serial.println("OLED: OK");
        showBootScreen();
    }

    // --- LoRa SX1262 Init ---
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    Serial.print("SX1262 init... ");
    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8,
                            1.8, false);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK");
    } else {
        Serial.printf("FAILED (error %d)\n", state);
    }

    // --- Initialize subsystems ---
    cwInit(&radio);
    fpInit(&radio);
    ridInit();
    combinedInit(&radio);
    swarmInit();
    crossfireInit(&radio);
    powerRampInit(&radio);
    menuInit(&display);

    // Hold boot screen for 2 seconds so user can read it
    delay(2000);

    // --- Ready ---
    Serial.println();
    Serial.println("JAMMER-RF ready.");
    Serial.println("Press BOOT button to navigate.");
    Serial.println("  SHORT press = cycle menu");
    Serial.println("  LONG  press = select/confirm");
    Serial.println();
    Serial.println("Serial commands:");
    Serial.println("  c = CW tone mode");
    Serial.println("  e = ELRS 200Hz FCC915 (default)");
    Serial.println("  e1-e6 = ELRS rate (200/100/50/25/D250/D500)");
    Serial.println("  e1f/a/u/i = ELRS domain (FCC/AU/EU/IN)");
    Serial.println("  e1fb = ELRS binding→connected sequence");
    Serial.println("  b = Band sweep mode");
    Serial.println("  r = RID spoofer (WiFi+BLE)");
    Serial.println("  m = Mixed false positive (LoRaWAN+ELRS)");
    Serial.println("  x = Combined (RID + ELRS dual-core)");
    Serial.println("  w = Drone swarm simulator");
    Serial.println("  p = cycle TX power");
    Serial.println("  d = cycle dwell time (sweep)");
    Serial.println("  s = cycle step size (sweep)");
    Serial.println("  n = cycle swarm drone count (1/4/8/16)");
    Serial.println("  q = stop TX and return to menu");

    digitalWrite(LED_PIN, LOW);
}

// --- Stop whatever mode is currently active ---
static void stopCurrentMode() {
    AppState st = menuGetState();
    if (st == STATE_CW_ACTIVE)       cwStop();
    if (st == STATE_SWEEP_ACTIVE)    sweepStop();
    if (st == STATE_ELRS_ACTIVE)     elrsStop();
    if (st == STATE_FP_ACTIVE)       fpStop();
    if (st == STATE_RID_ACTIVE)      ridStop();
    if (st == STATE_COMBINED_ACTIVE) combinedStop();
    if (st == STATE_SWARM_ACTIVE)    swarmStop();
    if (st == STATE_CROSSFIRE_ACTIVE) crossfireStop();
    if (st == STATE_RAMP_ACTIVE)     powerRampStop();
}

// --- Serial command parser ---
static void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    AppState st = menuGetState();

    switch (cmd) {
    case 'd':
        if (st == STATE_SWEEP_ACTIVE) {
            sweepCycleDwell();
        } else {
            Serial.println("(dwell only applies in sweep mode)");
        }
        break;

    case 's':
        if (st == STATE_SWEEP_ACTIVE) {
            sweepCycleStep();
        } else {
            Serial.println("(step only applies in sweep mode)");
        }
        break;

    case 'p':
        rfCyclePower();
        break;

    case 'q':
        stopCurrentMode();
        menuSetState(STATE_MAIN_MENU);
        Serial.println("TX stopped via serial.");
        break;

    // --- Direct mode selection via serial ---
    case 'c':   // CW Tone
        stopCurrentMode();
        cwStart();
        menuSetState(STATE_CW_ACTIVE);
        { CwParams p = cwGetParams();
          Serial.printf("[MODE] CW Tone: %.2f MHz, %d dBm\n", p.freqMHz, p.powerDbm); }
        break;

    case 'e': { // ELRS FHSS — e[1-6][f/a/u/i][b]
        // Parse optional rate digit, domain letter, binding flag
        delay(80);  // allow multi-char command to arrive
        uint8_t rateIdx = 0;   // default: 200 Hz
        uint8_t domIdx  = ELRS_DOMAIN_FCC915;  // default: FCC915
        bool    binding = false;

        // Rate digit (e1-e6)
        if (Serial.available()) {
            char c = Serial.peek();
            if (c >= '1' && c <= '6') {
                Serial.read();
                rateIdx = c - '1';
            }
        }
        delay(20);

        // Domain letter (f/a/u/i)
        if (Serial.available()) {
            char c = Serial.peek();
            switch (c) {
            case 'f': Serial.read(); domIdx = ELRS_DOMAIN_FCC915; break;
            case 'a': Serial.read(); domIdx = ELRS_DOMAIN_AU915;  break;
            case 'u': Serial.read(); domIdx = ELRS_DOMAIN_EU868;  break;
            case 'i': Serial.read(); domIdx = ELRS_DOMAIN_IN866;  break;
            default: break;  // no domain letter → keep default
            }
        }
        delay(20);

        // Binding flag (b)
        if (Serial.available() && Serial.peek() == 'b') {
            Serial.read();
            binding = true;
        }

        stopCurrentMode();
        elrsSetRate(rateIdx);
        elrsSetDomain(domIdx);
        if (binding) {
            elrsStartBinding();
        } else {
            elrsStart();
        }
        menuSetState(STATE_ELRS_ACTIVE);
        break;
    }

    case 'b':   // Band Sweep
        stopCurrentMode();
        sweepStart();
        menuSetState(STATE_SWEEP_ACTIVE);
        { SweepParams sw = sweepGetParams();
          Serial.printf("[MODE] Band Sweep: %.1f-%.1f MHz, step %.0f kHz, %d dBm\n",
                        sw.startMHz, sw.endMHz, sw.stepMHz * 1000.0f, sw.powerDbm); }
        break;

    case 'r':   // Remote ID Spoofer
        stopCurrentMode();
        ridStart();
        menuSetState(STATE_RID_ACTIVE);
        Serial.println("[MODE] RID Spoofer: WiFi+BLE beacons");
        break;

    case 'm':   // Mixed False Positive (LoRaWAN + ELRS)
        stopCurrentMode();
        fpStart(FP_MIXED);
        menuSetState(STATE_FP_ACTIVE);
        Serial.printf("[MODE] Mixed FP (LoRaWAN+ELRS): %d dBm\n", rfGetPower());
        break;

    case 'x':   // Combined (RID + ELRS dual-core)
        stopCurrentMode();
        combinedStart();
        menuSetState(STATE_COMBINED_ACTIVE);
        Serial.println("[MODE] Combined: RID(Core0) + ELRS(Core1)");
        break;

    case 'w':   // Drone Swarm Simulator
        stopCurrentMode();
        swarmStart();
        menuSetState(STATE_SWARM_ACTIVE);
        { SwarmParams sp = swarmGetParams();
          Serial.printf("[MODE] Swarm: %d drones, WiFi RID beacons\n", sp.droneCount); }
        break;

    case 'n':   // Cycle swarm drone count
        swarmCycleCount();
        break;

    default:
        if (cmd > ' ') {
            Serial.printf("Unknown command: '%c'. Use c/e/b/r/m/x/w/n/p/q.\n", cmd);
        }
        break;
    }
}

void loop() {
    // State machine drives everything — menu, display, and mode execution
    menuUpdate();

    // Process serial commands for runtime parameter control
    handleSerialCommands();

    // Heartbeat blink: slow when idle, fast when transmitting
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    AppState st = menuGetState();
    bool txActive = (st == STATE_CW_ACTIVE || st == STATE_SWEEP_ACTIVE
                     || st == STATE_ELRS_ACTIVE || st == STATE_FP_ACTIVE
                     || st == STATE_RID_ACTIVE || st == STATE_COMBINED_ACTIVE
                     || st == STATE_SWARM_ACTIVE
                     || st == STATE_CROSSFIRE_ACTIVE
                     || st == STATE_RAMP_ACTIVE);
    unsigned long blinkRate = txActive ? 200 : 1000;

    if (millis() - lastBlink >= blinkRate) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }

    // yield() feeds the watchdog with minimal overhead (~1ms).
    // TX modes are rate-limited by SX1262 airtime (~6.7ms/hop at SF6/BW500),
    // so the 1ms yield cost is negligible vs the 6.7ms TX blocking.
    yield();
}
