# CLAUDE.md - Guidelines for Abracadabra Sense Arduino Project

## Build & Run Commands
- Build: `pio run`
- Upload: `pio run --target upload`
- Monitor: `pio device monitor`
- Clean: `pio run --target clean`
- List Devices: `pio device list`

## Code Style Guidelines
- **Formatting**: 2-space indentation, consistent bracing style
- **Naming**: camelCase for variables/functions, UPPER_SNAKE for constants
- **Includes**: Group standard libraries first, then third-party, then project-specific
- **Error Handling**: Use JSON formatted error messages for serial output
- **Documentation**: Comment functions with purpose and parameters
- **Memory Management**: Prefer StaticJsonDocument with appropriate sizing
- **Timing**: Use non-blocking patterns with millis() rather than delay()
- **Types**: Use explicit types (uint32_t vs unsigned long) where appropriate

## Libraries
- LSM6DS3 for IMU sensor access
- ArduinoJson for structured data serialization

## Hardware
- Target: XIAO BLE Sense (nRF52840)
- Sensor: LSM6DS3 IMU (accelerometer/gyroscope)