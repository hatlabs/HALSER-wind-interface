# CLAUDE.md

## Project Overview

HALSER wind interface firmware — an ESP32-C3 firmware that bridges an Autonnic A5120 ultrasonic wind instrument to NMEA 2000 and Signal K networks via the HALSER board.

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Architecture

### Data Flow

```
Autonnic A5120 (NMEA 0183, 4800 bit/s, GPIO 3 RX / GPIO 2 TX)
  → NMEA0183IOTask (dedicated FreeRTOS task)
  → WIMWVSentenceParser (apparent wind)
  → ApparentWindData (speed + angle)
  → N2kWindDataSender (100ms interval)
  → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
  → Signal K output (via WiFi/WebSocket)
  → SSD1306 OLED display (I2C, GPIO 6 SDA / GPIO 7 SCL)
```

### Source Layout

- `src/main.cpp` — Application entry point, wiring
- `src/sender/n2k_senders.h` — N2K wind data sender with value expiry
- `src/ssd1306_display.h/.cpp` — OLED display (hostname, IP, wind data)
- `src/autonnic_a5120_parser.h` — Autonnic proprietary sentence parser
- `src/autonnic_config.h` — Autonnic A5120 configuration (reference angle, damping, repetition rate)

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

## Dependencies

- SensESP 3.2.0 — IoT framework (WiFi, web UI, Signal K)
- SensESP/NMEA0183 — NMEA 0183 sentence parsing
- NMEA2000-library — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit NeoPixel — RGB LED control
- Adafruit SSD1306 — OLED display
- elapsedMillis — Timing utilities
