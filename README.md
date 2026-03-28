# JUH-MAK-IN JAMMER

**Open-source drone signal test suite for validating counter-UAS detection systems**

![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![Platform: ESP32-S3](https://img.shields.io/badge/Platform-ESP32--S3-blue.svg)
![Version: v1.1.0](https://img.shields.io/badge/Version-v1.1.0-orange.svg)

## What It Does

JUH-MAK-IN JAMMER is a **test instrument** that generates simulated drone RF signals to validate counter-UAS detection systems like [SENTRY-RF](https://github.com/Seaforged/Sentry-RF). It is not a jammer, not a weapon, and not for interference — it is a calibrated signal generator for controlled testing of drone detection hardware and algorithms.

Think of it as an RF function generator that speaks drone protocols: ELRS frequency-hopping, TBS Crossfire FSK, ASTM F3411 Remote ID beacons, and multi-drone swarm simulation. Every mode is remotely controllable via serial commands for automated test scripting.

## Operating Modes

| Mode | Serial Cmd | Type | Description |
|------|------------|------|-------------|
| **CW Tone** | `c` | Sub-GHz | Unmodulated carrier, 5 frequency presets (868-925 MHz) |
| **Band Sweep** | `b` | Sub-GHz | Linear sweep 860-930 MHz, configurable step/dwell |
| **ELRS 915 FHSS** | `e` | Sub-GHz | 80-channel LoRa SF6 BW500, 200 Hz frequency hopping |
| **Crossfire 915** | button | Sub-GHz | 100-channel FSK 85.1 kbaud, 50 Hz hopping |
| **Power Ramp** | button | Sub-GHz | Approach simulation: -9 to +22 dBm ELRS ramp over 30-300s |
| **Remote ID** | `r` | WiFi+BLE | ASTM F3411 compliant Open Drone ID beacon broadcasting |
| **Mixed False Positive** | `m` | Sub-GHz | LoRaWAN + ELRS time-division (tests false alarm rejection) |
| **Combined** | `x` | Dual-Core | Remote ID on Core 0 + ELRS FHSS on Core 1 simultaneously |
| **Drone Swarm** | `w` | WiFi | 1-16 virtual drones with unique IDs and circular flight paths |

## Serial Command Reference

All modes are controllable via serial at 115200 baud. Connect via `pio device monitor` or any serial terminal.

| Command | Action |
|---------|--------|
| `c` | Start CW Tone mode |
| `e` | Start ELRS 915 FHSS mode |
| `b` | Start Band Sweep mode |
| `r` | Start Remote ID Spoofer (WiFi + BLE) |
| `m` | Start Mixed False Positive mode |
| `x` | Start Combined mode (RID + ELRS dual-core) |
| `w` | Start Drone Swarm Simulator |
| `n` | Cycle swarm drone count (1 / 4 / 8 / 16) |
| `p` | Cycle TX power (-9, 0, 5, 10, 14, 17, 20, 22 dBm) |
| `d` | Cycle dwell time (sweep mode only) |
| `s` | Cycle step size (sweep mode only) |
| `q` | Stop all TX, return to menu |

The OLED menu system provides the same functionality via button navigation (short press = navigate, long press = select).

## Hardware

| Component | Details |
|-----------|---------|
| Board | LilyGo T3S3 v1.3 (ESP32-S3 + SX1262) |
| Radio | Semtech SX1262, 150 MHz - 960 MHz, up to +22 dBm |
| Display | SSD1306 128x64 OLED via I2C |
| Antenna | SMA connector, 868/915 MHz antenna |
| Power | USB-C, 5V |

Same board as SENTRY-RF — you can flash either firmware to the same hardware.

## Quick Start

```bash
# Clone
git clone https://github.com/Seaforged/Juh-Mak-In-Jammer.git
cd Juh-Mak-In-Jammer

# Build
pio run -e jammer_rf

# Flash
pio run -e jammer_rf -t upload

# Monitor
pio device monitor -b 115200
```

On boot, the OLED shows the JUH-MAK-IN JAMMER splash screen for 2 seconds, then the main menu. Use the serial commands above or the BOOT button to select a mode.

## Automated Testing

The JAMMER is designed for automated dual-device testing against SENTRY-RF. Python test scripts are provided in the [SENTRY-RF repository](https://github.com/Seaforged/Sentry-RF):

- **`run_test.py <mode>`** — Run a single JAMMER mode and read SENTRY-RF's detection response
- **`threshold_test.py`** — Measure detection probability (Pd) vs TX power across all power levels
- **`full_validation.py`** — Run all modes sequentially and generate a PASS/FAIL summary

Example: Connect JAMMER on COM6 and SENTRY-RF on COM8, then:

```bash
python run_test.py c   # Test CW tone detection
python run_test.py e   # Test ELRS FHSS detection
python run_test.py w   # Test drone swarm detection
```

## Part of the Seaforged Test Suite

JUH-MAK-IN JAMMER is the RF signal generation component of the Seaforged counter-UAS test infrastructure:

| Component | Role | Status |
|-----------|------|--------|
| [SENTRY-RF](https://github.com/Seaforged/Sentry-RF) | Passive drone detector (device under test) | Active |
| **JUH-MAK-IN JAMMER** | RF signal generator (test instrument) | Active |
| JAMMER-GPS (AntSDR E200) | GNSS spoofing/jamming simulator | Coming soon |

## Legal Disclaimer

This tool generates RF signals on ISM bands (868/915 MHz) and WiFi (2.4 GHz) for **testing purposes only**. Users are responsible for compliance with local RF regulations.

- Use in a shielded environment or with inline attenuators for conducted testing
- Do not transmit on frequencies you are not authorized to use
- This tool does not jam, interfere with, or disrupt any communication system
- The Remote ID spoofer generates test beacons only — do not use to impersonate real aircraft

The authors assume no liability for misuse. See [LICENSE](LICENSE) for details.

## License

MIT License. See [LICENSE](LICENSE) for details.
