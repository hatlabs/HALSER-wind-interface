# HALSER Wind Interface

ESP32-C3 firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) board that bridges an **Autonnic A5120** ultrasonic wind instrument to NMEA 2000 and Signal K networks.

## Features

- Receives apparent wind data (speed and angle) from the Autonnic A5120 via NMEA 0183 MWV sentences at 4800 bit/s
- Transmits wind data as NMEA 2000 PGN 130306 (Wind Data) at 100 ms intervals
- Outputs wind data to Signal K via WiFi/WebSocket
- Configurable Autonnic A5120 parameters via web UI:
  - Reference angle recalibration (one-shot, not stored)
  - Wind direction damping
  - Wind speed damping
  - Message repetition rate
- OLED display showing hostname, IP, uptime, wind speed, and wind angle
- RGB LED activity indicator
- OTA firmware updates
- NMEA 2000 watchdog with configurable auto-reboot
- Counting N2K bus wrapper (TX counter on the status page)
- Heap diagnostics (largest free block, main-loop stack headroom)

## Hardware Required

- [HALSER](https://shop.hatlabs.fi/products/halser) board
- [Autonnic A5120](https://autonnic.com/a5120/) ultrasonic wind instrument
- NMEA 2000 network connection
- Optional: SSD1306 128x64 OLED display (I2C)

## Wiring

| HALSER Pin | Function |
|------------|----------|
| GPIO 2 | UART1 TX → Autonnic RX |
| GPIO 3 | UART1 RX ← Autonnic TX |
| GPIO 4 | CAN TX → NMEA 2000 |
| GPIO 5 | CAN RX ← NMEA 2000 |
| GPIO 6 | I2C SDA (OLED display) |
| GPIO 7 | I2C SCL (OLED display) |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

The Autonnic A5120 communicates via NMEA 0183 at 4800 bit/s (8N1).

### Hardware Connection

The A5120 uses RS-232 levels for its TX output (data to HALSER) and NMEA 0183 levels for its RX input (configuration commands from HALSER). Set the HALSER RX jumper to **R** (RS-232 mode). HALSER TX is connected to the NMEA 0183 TX output.

Use a 5-pin SP13 connector to route the masthead cable into the HALSER enclosure.

## Building

Requires [PlatformIO](https://platformio.org/).

The project defines two environments:

| Environment | Framework | Use |
|-------------|-----------|-----|
| `halser` | arduino (pioarduino) | Quick compile checks. Builds in seconds. |
| `halser_espidf` | espidf + arduino | **Always flash this.** Builds ESP-IDF from source so `sdkconfig.defaults` is authoritative. |

The `halser_espidf` env is `default_envs`, so plain `pio run` builds the right thing.

**Do not flash the `halser` env to a device that talks to a TLS-enabled Signal K server.** The arduino env uses precompiled libs that ignore `sdkconfig.defaults`, so the dynamic mbedTLS buffer is inactive. The device boots and joins WiFi but Signal K stays Disconnected (`mbedtls_ssl_setup` fails with `-0x7F00`).

```bash
# Build (uses default_envs = halser_espidf)
pio run

# Upload to connected board
pio run -t upload

# Arduino compile check
pio run -e halser

# Run unit tests (native platform)
pio test -e native
```

The first `halser_espidf` build downloads the ESP-IDF toolchain (several hundred MB) and takes minutes. On Windows, use a short project path without spaces.

### OTA Password

The OTA password in `src/main.cpp` is a placeholder (`change-me`). Change it before deploying to a device.

### sdkconfig

ESP-IDF reads `sdkconfig.defaults` on the first build and generates `sdkconfig.halser_espidf` in the project root. That generated file overrides the defaults on every later build and survives `pio run -t fullclean`. After editing `sdkconfig.defaults`, delete `sdkconfig.halser_espidf` and rebuild:

```bash
rm -f sdkconfig.halser_espidf
pio run
```

## Usage

### Initial Setup

1. Flash the `halser_espidf` firmware to the HALSER board
2. The device creates a WiFi access point on first boot
3. Connect to the AP and configure your WiFi network credentials
4. Access the web UI at `http://wind.local`

### Reference Angle

Navigate to the **Reference Angle** card in the web UI. Enter the angle the vane should report for its current physical position, in degrees from -180 to 180 (0 = dead ahead, 180 = dead astern).

This is a one-shot command: the value is sent to the Autonnic as a `$PATC,IIMWV,AHD` recalibration and is not stored. The field always reads 0. Re-applying the same value is safe.

The ±180° range is enforced in firmware. The SensESP number input does not honor JSON schema minimum/maximum, and an out-of-range value is silently bounced by the sensor, so the firmware rejects it before commanding a recalibration.

### Damping

The **Wind Direction Damping** and **Wind Speed Damping** cards control smoothing applied to the wind data. Values range from 0 to 100, with a default of 50. Higher values produce smoother readings but increase response lag.

### Message Repetition Rate

The **Message Repetition Rate** card sets how often the A5120 sends MWV sentences, in milliseconds. Default is 500 ms. Changing this value causes the Autonnic to pause output briefly before acknowledging.

### NMEA 2000 Watchdog

An optional watchdog can be enabled under **Enable NMEA 2000 Watchdog**. When enabled, the device reboots if no NMEA 2000 messages are received for two minutes. This helps recover from CAN bus lockups. The setting requires a device restart to take effect.

### Signal K Integration

Wind data is emitted to Signal K as:

- `environment.wind.speedApparent` — apparent wind speed in m/s
- `environment.wind.angleApparent` — apparent wind angle in radians (signed: negative to port)

### NMEA 2000 Stale Data Handling

The N2K sender uses `RepeatExpiring` to handle stale wind data. If no new wind measurement arrives within 5 seconds, the sender transmits `N2kDoubleNA` ("not available") values instead of repeating stale data. PGN 130306 messages continue at 100 ms regardless — downstream devices always see a consistent message rate and can distinguish "no data" from silence.

### OLED Display

If connected, a 128x64 SSD1306 OLED display shows:
- Device hostname
- WiFi IP address
- Uptime in seconds
- Apparent wind speed (m/s)
- Apparent wind angle (degrees, -180 to +180)

## Architecture

```
Autonnic A5120 (NMEA 0183, 4800 bit/s)
  │
  │ Serial1, 1024-byte RX buffer (GPIO 3 RX / GPIO 2 TX)
  ▼
NMEA0183IO (reads on the main ReactESP event loop)
  ├── MWVSentenceParser (apparent wind speed + angle)
  │     ├── N2kWindDataSender → CountingNMEA2000 (TWAI, GPIO 4/5)
  │     ├── SKOutputFloat     → Signal K server (speed + signed angle)
  │     └── InfoDisplay       → OLED (speed + angle)
  │
  └── AutonnicPATCWIMWVParser (ACK responses for config commands)

Web UI (SensESP) ──── Config objects ──── Autonnic (serial commands)
                                     └── Filesystem (persistent storage)
```

The NMEA 0183 input is read on the main ReactESP event loop, not a separate FreeRTOS task. The ESP32-C3 is single-core, so a reader task buys no parallelism and only adds a cross-task propagation hazard. `NMEA0183IO` keeps the whole pipeline single-threaded.

The firmware is built on [SensESP](https://github.com/SignalK/SensESP), which provides WiFi connectivity, a web UI for configuration, Signal K protocol support, and OTA updates.

### NMEA 2000 Device Identity

| Field | Value |
|-------|-------|
| Device function | 130 (Weather Instruments) |
| Device class | 85 (Sensor Communication Interface) |
| Manufacturer code | 2046 |
| Default source address | 72 |
| Transmitted PGN | 130306 (Wind Data) |
| Wind reference | Apparent |

## Testing

```bash
pio test -e native
```

Runs the reference-angle range bound tests: accepts 0, ±pi, ±180° via the web UI's deg→rad path; rejects values outside ±180° and non-finite inputs (NaN, ±Infinity).

## Upgrading

### From the pre-2026 firmware (single-env, NMEA0183IOTask)

The 2026 update changes three things:

1. **Two-env build layout.** The old single `halser` env ran `framework = espidf, arduino`. It is now the arduino compile-check env; the flashable env is `halser_espidf`. Plain `pio run` builds the right one (`default_envs = halser_espidf`).

2. **Main-loop NMEA reader.** `NMEA0183IOTask` (a separate FreeRTOS task) is replaced by `NMEA0183IO` (reads on the ReactESP event loop). The C3 is single-core, so the task bought no parallelism.

3. **CountingNMEA2000.** The bus object counts TX messages itself. The old `ValueProducer` emit-on-send pattern in `N2kWindDataSender` and the manual TX counter are gone.

### From the arduino USB-serial build

If the device was last flashed with the arduino env (pre-2026 default), the first flash of `halser_espidf` switches the USB descriptor from the Arduino CDC to the ESP32-C3 USB-Serial-JTAG. The serial port name changes (e.g. from `/dev/cu.usbmodemXXXX` to a different `/dev/cu.usbmodemYYYY` on macOS). The device is the same; only the port enumerator changed.

## License

See [LICENSE](LICENSE) for details.
