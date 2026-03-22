# CLAUDE.md

## Project Overview

HALSER wind interface firmware — an ESP32-C3 firmware that bridges an Autonnic A5120 ultrasonic wind instrument to NMEA 2000 and Signal K networks via the HALSER board. Also serves as a reference implementation for SensESP-based marine interface firmware.

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor

# Run unit tests (native platform)
pio test -e native
```

## Architecture

### Data Flow

```
Autonnic A5120 (NMEA 0183, 4800 bit/s, GPIO 3 RX / GPIO 2 TX)
  → NMEA0183IOTask (dedicated FreeRTOS task)
    → WIMWVSentenceParser (apparent wind speed + angle)
      → ApparentWindData
        → N2kWindDataSender (PGN 130306, 100ms interval)
          → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
        → Signal K output (via WiFi/WebSocket)
        → SSD1306 OLED display (hostname, IP, uptime, AWS, AWA)
    → AutnnicA5120Parser (ACK responses for config commands)

Web UI ←→ Autonnic config objects ←→ Autonnic A5120 (serial commands)
```

### Source Layout

**Autonnic Configuration** (`src/`):
- `autonnic_config.h` — 4 FileSystemSaveable config classes that persist to flash and send commands to the Autonnic via UART; uses SemaphoreValue-based ACK confirmation
- `autonnic_a5120_parser.h` — SentenceParser for proprietary `$PATC,WIMWV` ACK responses (ignores checksum because Autonnic omits it)

**NMEA 2000 Output** (`src/sender/`):
- `n2k_senders.h` — Wind data sender: PGN 130306 at 100ms interval, uses RepeatExpiring (5s timeout) to send N2kDoubleNA for stale data; also a ValueProducer that emits on TX for downstream consumers

**Application** (`src/`):
- `main.cpp` — Entry point; initializes all components and wires the data pipeline
- `ssd1306_display.h/.cpp` — OLED display driver (hostname, IP, uptime, AWS, AWA; updates every 1 second)

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

### Autonnic A5120 Protocol

Configuration uses proprietary NMEA 0183 sentences. Commands use talker ID `II` (integrated instrumentation), responses use `WI` (weather instruments).

**Reference angle:**
- Set: `$PATC,IIMWV,AHD,<degrees>` → Response: `$PATC,WIMWV,ACK`

**Direction damping:**
- Set: `$PATC,IIMWV,DWD,<factor>` → Response: `$PATC,WIMWV,ACK`

**Speed damping:**
- Set: `$PATC,IIMWV,DSP,<factor>` → Response: `$PATC,WIMWV,ACK`

**Repetition rate:**
- Set: `$PATC,IIMWV,TXP,<ms>` → Response: `$PATC,WIMWV,ACK`

ACK responses have no checksum — the parser skips checksum validation for these sentences.

### NMEA 2000 PGNs

| PGN | Description | Interval |
|-----|-------------|----------|
| 130306 | Wind Data (apparent wind speed + angle) | 100ms |

## Dependencies

- SensESP 3.2.0 — IoT framework (WiFi, web UI, Signal K)
- SensESP/NMEA0183 — NMEA 0183 sentence parsing
- NMEA2000-library v4.17.2 — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- FastLED 3.9.4 — RGB LED (SK6805, managed by SensESP)
- Adafruit SSD1306 v2.5.1 — OLED display
- elapsedMillis v1.0.6 — Timing utilities
- esp_websocket_client — WebSocket support (Espressif component)
