# Abracadabra Sense Arduino

A sensor data collection system using the Seeed XIAO BLE Sense board (nRF52840).

## Features

- Reads accelerometer and gyroscope data from the LSM6DS3 IMU sensor
- Formats sensor data as JSON
- Sends formatted data over serial connection at 10Hz
- Toggles the onboard LED as a visual indicator

## Project Structure

- `src/config/` - Configuration parameters
- `src/sensors/` - Sensor interfaces
- `src/utils/` - Utility classes
- `src/communication/` - Communication and serialization

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/)
- [Seeed XIAO BLE Sense](https://wiki.seeedstudio.com/XIAO_BLE/) board

### Building and Uploading

```bash
# Build the project
pio run

# Upload to the board
pio run --target upload

# Monitor serial output
pio device monitor
```

## License

[MIT](LICENSE)