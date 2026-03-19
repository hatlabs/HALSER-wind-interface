# HALSER Wind Interface

ESP32-C3 firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) board that bridges an **Autonnic A5120** ultrasonic wind instrument to NMEA 2000 and Signal K networks.

## Features

- Receives apparent wind data (speed and angle) via NMEA 0183 at 4800 bit/s
- Transmits wind data as NMEA 2000 PGN 130306 (Wind Data) at 100ms intervals
- Outputs wind data to Signal K via WiFi/WebSocket
- Configurable Autonnic A5120 parameters via web UI:
  - Reference angle offset
  - Wind direction damping
  - Wind speed damping
  - Message repetition rate
- OLED display showing hostname, IP, wind speed, and wind angle
- RGB LED activity indicator
- OTA firmware updates
- NMEA 2000 watchdog (optional)

## Hardware Required

- [HALSER](https://shop.hatlabs.fi/products/halser) board
- [Autonnic A5120](https://autonnic.com/a5120/) ultrasonic wind instrument
- NMEA 2000 network connection
- Optional: SSD1306 128x64 OLED display (I2C)

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run -t upload

# Monitor serial output
pio device monitor
```

## License

See [LICENSE](LICENSE) for details.
