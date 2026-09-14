# AGENTS.md

## Project Overview

HALSER wind interface firmware — an ESP32-C3 firmware that bridges an Autonnic A5120 ultrasonic wind instrument to NMEA 2000 and Signal K networks via the HALSER board.

## Build Commands

```bash
# Build firmware (default_envs = halser_espidf)
pio run

# Arduino compile check
pio run -e halser

# Upload to connected board
pio run -t upload

# Run unit tests (native platform)
pio test -e native
```

## Architecture

### Data Flow

```
Autonnic A5120 (NMEA 0183, 4800 bit/s, GPIO 3 RX / GPIO 2 TX)
  → NMEA0183IO (reads on the main ReactESP event loop)
    → MWVSentenceParser (apparent wind speed + angle)
      → N2kWindDataSender (PGN 130306, 100 ms interval, NaN guard)
          → CountingNMEA2000 (TWAI, GPIO 4 TX / GPIO 5 RX)
      → Signal K output (via WiFi/WebSocket, signed angle)
      → SSD1306 OLED display (hostname, IP, uptime, AWS, AWA)
    → AutonnicPATCWIMWVParser (ACK responses for config commands)

Web UI ←→ Autonnic config objects ←→ Autonnic A5120 (serial commands)
```

### Source Layout

**Autonnic Configuration** (`src/`):
- `autonnic_config.h` — `AutonnicFloatConfig` (reusable parameterized base), `AutonnicReferenceAngleConfig` (write-only, ±180° bound), `WindOutputRepetitionRateConfig`. All take `Stream*` for UART output and defer writes to the event loop via `onDelay(0)`
- `autonnic_a5120_parser.h` — SentenceParser for proprietary `$PATC,WIMWV` ACK responses (ignores checksum because Autonnic omits it)
- `reference_angle.h` — Pure, hardware-independent ±180° bound with 1e-6 rad slack for the web UI's float round-trip; host-tested

**NMEA 2000 Output** (`src/sender/`):
- `n2k_senders.h` — `N2kSender` base, `N2kWindDataSender`: PGN 130306 at 100 ms with `RepeatExpiring` (5 s timeout), NaN→N2kDoubleNA guard. Takes `CountingNMEA2000*`

**Application** (`src/`):
- `main.cpp` — Entry point; wires the data pipeline. OTA password is a placeholder (`change-me`)
- `counting_nmea2000.h` — `tNMEA2000_esp32` subclass that counts accepted `SendMsg` calls; senders take `CountingNMEA2000*` (not `tNMEA2000*`) because `SendMsg` is not virtual
- `ssd1306_display.h/.cpp` — OLED display driver (hostname, IP, uptime, AWS, AWA; updates every 1 s)

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 2 | UART1 TX (to Autonnic) |
| GPIO 3 | UART1 RX (from Autonnic) |
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 6 | I2C SDA |
| GPIO 7 | I2C SCL |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

### NMEA 2000 PGNs

| PGN | Description | Interval |
|-----|-------------|----------|
| 130306 | Wind Data (apparent wind speed + angle) | 100 ms |

Default source address: 72.

### Signal K Paths

- `environment.wind.speedApparent` — apparent wind speed in m/s
- `environment.wind.angleApparent` — apparent wind angle in radians (signed: negative to port)

## Dependencies

- SensESP ^3.5.0 — IoT framework (WiFi, web UI, Signal K)
- SensESP/NMEA0183 ^3.2.0 — NMEA 0183 sentence parsing (NMEA0183IO reader)
- NMEA2000-library ^4.17.2 — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit SSD1306 ^2.5.1 — OLED display
- elapsedMillis ^1.0.6 — Timing utilities
- esp_websocket_client 1.7.0 — WebSocket support (Espressif component, per-env sourcing)
