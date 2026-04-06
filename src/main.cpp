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
#include "sik_radio.h"
#include "mlrs_sim.h"
#include "custom_lora.h"
#include "infra_sim.h"
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

// --- Help Menu ---
static void printHelp() {
    Serial.printf("=== %s v%s — Drone Signal Emulator ===\n\n", JAMMER_NAME, JAMMER_VERSION);

    Serial.println("DRONE PROTOCOLS:");
    Serial.println("  e  ELRS FHSS      e1-e6=rate  f/a/u/i=domain  b=binding");
    Serial.println("  k  SiK Radio      k1=64k  k2=125k  k3=250k");
    Serial.println("  l  mLRS           l1=19Hz  l2=31Hz  l3=50Hz(FSK)");
    Serial.println("  u  Custom LoRa    u?=settings  uf/us/ub/ur/uh/up/uw=config");

    Serial.println("\nINFRASTRUCTURE (False Positive Testing):");
    Serial.println("  m  LoRaWAN US915  Mixed FP (8 SB2 channels + ELRS)");
    Serial.println("  f1 Meshtastic     16-sym preamble, sync 0x2B");
    Serial.println("  f2 Helium PoC     5 hotspots, rotating SB2 channels");
    Serial.println("  f3 LoRaWAN EU868  868.1/868.3/868.5 MHz");

    Serial.println("\nSPECIAL MODES:");
    Serial.println("  c  CW Tone        b=sweep  w=power ramp  x=combined");
    Serial.println("  r  Remote ID      WiFi+BLE ASTM F3411 broadcast");
    Serial.println("  w  Drone Swarm    n=cycle count (1/4/8/16)");

    Serial.println("\nCONTROLS:");
    Serial.println("  q  Stop TX        p=cycle power  h/?=this menu");
    Serial.println();
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
    sikInit(&radio);
    mlrsInit(&radio);
    customLoraInit(&radio);
    infraInit(&radio);
    menuInit(&display);

    // Hold boot screen for 2 seconds so user can read it
    delay(2000);

    // --- Ready ---
    Serial.println();
    printHelp();

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
    if (st == STATE_SIK_ACTIVE)      sikStop();
    if (st == STATE_MLRS_ACTIVE)     mlrsStop();
    if (st == STATE_CUSTOM_LORA_ACTIVE) customLoraStop();
    if (st == STATE_INFRA_ACTIVE)    infraStop();
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

    case 'k': { // SiK Radio GFSK — optional digit selects air speed
        delay(50);
        uint8_t speedIdx = 1;  // default: 64 kbps (index 1 in SIK_AIR_SPEEDS_KBPS)
        if (Serial.available()) {
            char c = Serial.peek();
            if (c >= '1' && c <= '3') {
                Serial.read();
                speedIdx = c - '1' + 1;  // '1'→1(64k), '2'→2(125k), '3'→3(250k)
            }
        }
        stopCurrentMode();
        sikSetSpeed(speedIdx);
        sikStart();
        menuSetState(STATE_SIK_ACTIVE);
        break;
    }

    case 'l': { // mLRS — optional digit selects mode
        delay(50);
        uint8_t modeIdx = 0;  // default: 19 Hz LoRa
        if (Serial.available()) {
            char c = Serial.peek();
            if (c >= '1' && c <= '3') {
                Serial.read();
                modeIdx = c - '1';  // '1'→0(19Hz), '2'→1(31Hz), '3'→2(50Hz)
            }
        }
        stopCurrentMode();
        mlrsSetMode(modeIdx);
        mlrsStart();
        menuSetState(STATE_MLRS_ACTIVE);
        break;
    }

    case 'f': { // Infrastructure false positive modes — f1/f2/f3
        delay(50);
        if (Serial.available()) {
            char c = Serial.read();
            stopCurrentMode();
            switch (c) {
            case '1': infraStart(INFRA_MESHTASTIC); break;
            case '2': infraStart(INFRA_HELIUM_POC); break;
            case '3': infraStart(INFRA_LORAWAN_EU); break;
            default:
                Serial.println("f1=Meshtastic f2=Helium f3=LoRaWAN-EU868");
                break;
            }
            if (c >= '1' && c <= '3') menuSetState(STATE_INFRA_ACTIVE);
        } else {
            Serial.println("f1=Meshtastic f2=Helium f3=LoRaWAN-EU868");
        }
        break;
    }

    case 'u': { // Custom LoRa — u=start, u?=show, uXvalue=configure
        delay(80);
        if (!Serial.available()) {
            // Bare 'u' — start transmission
            stopCurrentMode();
            customLoraStart();
            menuSetState(STATE_CUSTOM_LORA_ACTIVE);
        } else {
            // Read subcommand + value into buffer
            char buf[16];
            uint8_t len = 0;
            unsigned long deadline = millis() + 100;
            while (len < sizeof(buf) - 1 && millis() < deadline) {
                if (Serial.available()) {
                    char c = Serial.read();
                    if (c == '\n' || c == '\r') break;
                    buf[len++] = c;
                    deadline = millis() + 50;  // extend for more chars
                }
            }
            buf[len] = '\0';

            if (buf[0] == '?') {
                customLoraPrintConfig();
            } else if (len > 0) {
                customLoraConfigure(buf);
            }
        }
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

    case 'h':
    case '?':
        printHelp();
        break;

    default:
        if (cmd > ' ') {
            Serial.printf("Unknown command: '%c'. Type 'h' for help.\n", cmd);
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
                     || st == STATE_RAMP_ACTIVE
                     || st == STATE_SIK_ACTIVE
                     || st == STATE_MLRS_ACTIVE
                     || st == STATE_CUSTOM_LORA_ACTIVE
                     || st == STATE_INFRA_ACTIVE);
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
