# HALSER Wind Interface

ESP32-C3 firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) board that bridges an **Autonnic A5120** ultrasonic wind instrument to NMEA 2000 and Signal K networks.

This firmware serves as both a ready-to-use application and a reference example for building custom SensESP-based marine interface firmware.

## Features

- Receives apparent wind data (speed and angle) from the Autonnic A5120 via NMEA 0183 WIMWV sentences at 4800 bit/s
- Transmits wind data as NMEA 2000 PGN 130306 (Wind Data) at 100ms intervals
- Outputs wind data to Signal K via WiFi/WebSocket
- Configurable Autonnic A5120 parameters via web UI:
  - Reference angle offset (wind vane alignment)
  - Wind direction damping
  - Wind speed damping
  - Message repetition rate
- OLED display showing hostname, IP, uptime, wind speed, and wind angle
- RGB LED activity indicator
- OTA firmware updates
- NMEA 2000 watchdog with configurable auto-reboot

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

## Usage

### Initial Setup

1. Flash the firmware to the HALSER board
2. The device creates a WiFi access point on first boot
3. Connect to the AP and configure your WiFi network credentials
4. Access the web UI at `http://wind.local`

### Configuring Reference Angle

Navigate to the **Reference Angle** section in the web UI. Enter the angle readout (in degrees) when the wind vane is pointing straight ahead. This offset corrects for misalignment between the wind instrument and the vessel's heading.

The value is stored internally in radians but displayed in degrees in the web UI.

### Configuring Damping

The **Wind Direction Damping** and **Wind Speed Damping** sections control smoothing applied to the wind data. Values range from 0 to 100, with a default of 50. Higher values produce smoother readings but increase response lag.

### Configuring Message Repetition Rate

The **Message Repetition Rate** sets how often the A5120 sends WIMWV sentences, in milliseconds. Default is 500ms.

### Dual Configuration Storage

All Autonnic configuration parameters are stored in two places:

1. **ESP32 filesystem** — persists across firmware reboots
2. **Autonnic A5120** — sent as proprietary `$PATC,IIMWV` commands with ACK confirmation

When a setting is saved via the web UI, the firmware writes to the filesystem and sends the corresponding command to the A5120. A semaphore-based mechanism waits for the `$PATC,WIMWV,ACK` response to confirm the command was accepted.

### Signal K Integration

Wind data is emitted to Signal K as:

- `environment.wind.speedApparent` — apparent wind speed in m/s
- `environment.wind.angleApparent` — apparent wind angle in radians

### NMEA 2000 Watchdog

An optional watchdog can be enabled in the web UI under **Enable NMEA 2000 Watchdog**. When enabled, the device reboots if no NMEA 2000 messages are received for two minutes. This helps recover from CAN bus lockups. The setting requires a device restart to take effect.

### NMEA 2000 Stale Data Handling

The N2K sender uses `RepeatExpiring` to handle stale wind data. If no new wind measurement arrives within 5 seconds, the sender transmits `N2kDoubleNA` ("not available") values instead of repeating stale data. PGN 130306 messages continue at 100ms regardless — downstream devices always see a consistent message rate and can distinguish "no data" from silence.

### OLED Display

If connected, a 128x64 SSD1306 OLED display shows:
- Device hostname
- WiFi IP address
- Uptime in seconds
- *(blank line)*
- Apparent wind speed (m/s)
- Apparent wind angle (degrees, -180 to +180)

## Architecture

```
Autonnic A5120 (NMEA 0183, 4800 bit/s)
  │
  │ UART1 (GPIO 3 RX / GPIO 2 TX)
  ▼
NMEA0183IOTask
  ├── WIMWVSentenceParser (apparent wind speed + angle)
  │     ├── N2kWindDataSender → NMEA 2000 bus (TWAI, GPIO 4/5)
  │     ├── SKOutputFloat     → Signal K server (speed + angle)
  │     └── InfoDisplay       → OLED (speed + angle)
  │
  └── AutonnicPATCWIMWVParser (ACK responses for config commands)

Web UI (SensESP) ──── Config objects ──── Autonnic (serial commands)
                                     └── Filesystem (persistent storage)
```

The firmware is built on [SensESP](https://github.com/SignalK/SensESP), which provides WiFi connectivity, a web UI for configuration, Signal K protocol support, and OTA updates.

### Key Design Patterns

**Producer/Consumer pipeline:** SensESP uses a reactive pipeline where producers emit values that flow to connected consumers. The WIMWV parser produces apparent wind speed and angle values consumed by the N2K sender, Signal K outputs, and OLED display. Producers and consumers are connected via `connect_to()`.

**Dual config storage with command/ACK confirmation:** Each config object (`ReferenceAngleConfig`, `WindDirectionDampingConfig`, etc.) saves to the ESP32 filesystem and sends a proprietary command to the Autonnic A5120. A `SemaphoreValue` waits up to 1 second (5 seconds for repetition rate) for the ACK response, parsed by `AutonnicPATCWIMWVParser`.

**NMEA 2000 value expiry:** The `N2kWindDataSender` wraps inputs in `RepeatExpiring<double>`, which returns `N2kDoubleNA` when the source value is older than 5 seconds. This prevents stale wind data from being transmitted as valid measurements while maintaining the 100ms PGN 130306 transmission rate.

## Code Structure

### Autonnic Configuration (`src/`)

| File | Purpose |
|------|---------|
| `autonnic_config.h` | 4 config classes (reference angle, direction damping, speed damping, repetition rate) with dual filesystem/serial storage |
| `autonnic_a5120_parser.h` | `SentenceParser` for proprietary `$PATC,WIMWV,ACK` responses |

### NMEA 2000 Output (`src/sender/`)

| File | Purpose |
|------|---------|
| `n2k_senders.h` | `N2kWindDataSender` — PGN 130306 at 100ms with `RepeatExpiring` for stale data |

### Application

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point — wires all components together |
| `ssd1306_display.h/.cpp` | OLED display driver (hostname, IP, uptime, AWS, AWA) |

### Autonnic A5120 Protocol

Configuration uses proprietary NMEA 0183 sentences:

| Command | Sentence | Description |
|---------|----------|-------------|
| Reference angle | `$PATC,IIMWV,AHD,<degrees>` | Wind vane alignment offset |
| Direction damping | `$PATC,IIMWV,DWD,<factor>` | Direction smoothing (0-100) |
| Speed damping | `$PATC,IIMWV,DSP,<factor>` | Speed smoothing (0-100) |
| Repetition rate | `$PATC,IIMWV,TXP,<ms>` | Message interval in milliseconds |

All commands receive a `$PATC,WIMWV,ACK` response on success. The parser ignores checksums because the A5120 does not include them in responses.

### NMEA 2000 Device Identity

| Field | Value |
|-------|-------|
| Device function | 130 (Weather Instruments) |
| Device class | 85 (Sensor Communication Interface) |
| Manufacturer code | 2046 |
| Transmitted PGN | 130306 (Wind Data) |
| Wind reference | Apparent |

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Testing

This repository does not yet have automated tests. Contributions are welcome.

## Using as a Template

This firmware demonstrates several patterns useful for building custom SensESP marine interfaces:

1. **NMEA 0183 sentence parsing** — Using SensESP's built-in `WIMWVSentenceParser` for standard sentences
2. **External device configuration** — Command/ACK pattern with semaphore-based confirmation and timeout handling
3. **NMEA 2000 output with value expiry** — `RepeatExpiring` prevents stale data transmission while maintaining constant PGN rate
4. **Dual config storage** — Persisting settings to both the ESP32 filesystem and the external device via serial commands
5. **Producer/Consumer pipeline** — Connecting a single data source (wind parser) to multiple sinks (N2K, Signal K, OLED)

To adapt this for a different device:
- Replace the `AutonnicPATCWIMWVParser` and config classes with your device's protocol
- Modify or replace `WIMWVSentenceParser` if your device uses different NMEA 0183 sentences
- Update the N2K sender for your target PGNs
- Adjust pin assignments and bit rate in `main.cpp`

## License

See [LICENSE](LICENSE) for details.
