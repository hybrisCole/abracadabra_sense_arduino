#ifndef CONFIG_H
#define CONFIG_H

// Serial configuration
const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long SERIAL_INIT_DELAY = 1000;

// Sensor configuration
const int IMU_I2C_ADDRESS = 0x6A;

// Battery monitoring configuration
const int BATTERY_PIN = A0;
const float BATTERY_VREF = 3.3;
const int BATTERY_SAMPLES = 10;

// Timing configuration
const unsigned long SENSOR_READ_INTERVAL = 500;  // in milliseconds - match OLED update interval

// Buffer sizes
const size_t JSON_DOCUMENT_SIZE = 512;

#endif