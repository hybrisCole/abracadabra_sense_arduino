#ifndef CONFIG_H
#define CONFIG_H

// Serial configuration
const unsigned long SERIAL_BAUD_RATE = 9600;
const unsigned long SERIAL_INIT_DELAY = 2000;

// Sensor configuration
const int IMU_I2C_ADDRESS = 0x6A;

// Timing configuration
const unsigned long SENSOR_READ_INTERVAL = 100;  // in milliseconds

// Buffer sizes
const size_t JSON_DOCUMENT_SIZE = 256;

#endif