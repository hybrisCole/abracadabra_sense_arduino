#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <stdint.h>
#include <algorithm>  // For std::min
#include <ArduinoBLE.h>

// Create IMU object - XIAO nRF52840 Sense has built-in LSM6DS3
LSM6DS3 imu(I2C_MODE);  // Using default I2C address 0x6A

// RGB LED pins for XIAO nRF52840 Sense
#define LED_RED    11
#define LED_GREEN  12
#define LED_BLUE   13

// Variables for tap detection debugging
unsigned long lastTapTime = 0;
unsigned long lastNonZeroReadTime = 0;
uint8_t lastTapSource = 0;
int tapCount = 0;
bool tapSequenceActive = false;
unsigned long tapSequenceStartTime = 0;

// Variables for software double tap detection
unsigned long lastSingleTapTime = 0;
bool doubleTapDetected = false;
#define DOUBLE_TAP_WINDOW 400  // Time window for double tap detection (ms)

// Flag to prevent tap detection during animations
bool ignoreTaps = false;

// Gesture recording state variables
bool isRecording = false;
unsigned long recordingStartTime = 0;
#define RECORDING_DURATION 4000  // Record for 4 seconds

// Sampling configuration - increasing to 250Hz for better gesture detail
#define SAMPLE_RATE_MS 4  // Sample every 4ms (approximately 250Hz)
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;

// Gesture storage constants
#define MAX_GESTURE_SAMPLES 1000     // Maximum samples in a processed gesture (250Hz * 4s)

// Gesture authentication settings
#define AUTH_THRESHOLD 0.75         // Similarity threshold for authentication (0.0-1.0)
#define NUM_AUTH_FEATURES 6         // Number of authentication features used
int authAttemptsCount = 0;          // Number of authentication attempts
bool authenticationMode = false;    // Whether we're in authentication or enrollment mode

// Gesture reference storage - using completely raw sensor data with no processing
typedef struct {
  float accX[MAX_GESTURE_SAMPLES];  // Raw accelerometer X values
  float accY[MAX_GESTURE_SAMPLES];  // Raw accelerometer Y values 
  float accZ[MAX_GESTURE_SAMPLES];  // Raw accelerometer Z values
  float gyroX[MAX_GESTURE_SAMPLES]; // Raw gyroscope X values
  float gyroY[MAX_GESTURE_SAMPLES]; // Raw gyroscope Y values
  float gyroZ[MAX_GESTURE_SAMPLES]; // Raw gyroscope Z values
  int sampleCount;                  // Number of samples stored
  bool isActive;                    // Whether this gesture is valid
  int gestureID;                    // Gesture type ID
  int repetition;                   // Which repetition this is
} GestureData;

GestureData referenceGestures[3];  // 3 reference gestures
GestureData currentGesture;        // Current gesture being recorded
int currentReferenceIndex = 0;     // Current reference gesture index (0-2)

// Near the top of the file, after other #define statements
#define DEVICE_ID "xiao_sense_001"  // Unique identifier for this device
#define ACCEL_SATURATION_THRESHOLD 8.0  // Threshold to detect potential sensor saturation (in g)
#define GYRO_SATURATION_THRESHOLD 1000.0  // Threshold for gyro saturation (in dps)

// BLE Configuration (for React Native app reference)
#define BLE_DEVICE_NAME "AbracadabraIMU"
#define BLE_SERVICE_UUID "8cfc8e26-0682-4f72-b0c0-c0c8e0b12a06"
#define BLE_DATA_CHAR_UUID "780fe2ec-c87c-443e-bf01-78918d9d625b"
#define BLE_COMMAND_CHAR_UUID "aa7e97b4-d7dc-4cb0-9fef-85875036520e"

// BLE Service and Characteristics
BLEService gestureService(BLE_SERVICE_UUID);
BLECharacteristic dataCharacteristic(BLE_DATA_CHAR_UUID, BLERead | BLENotify, 20); // 20 bytes max per notification
BLECharacteristic commandCharacteristic(BLE_COMMAND_CHAR_UUID, BLEWrite, 20);

// New global variables
char recordingId[20];  // Buffer to store unique recording ID
unsigned long absoluteStartTime;  // Store absolute timestamp when recording started

// Variables for consistent sampling timing
unsigned long nextSampleTime = 0;  // Next scheduled sample time
unsigned long sampleInterval = SAMPLE_RATE_MS;  // Sample interval in ms
unsigned long actualSampleInterval = 0;  // To track actual sampling intervals
unsigned long maxSampleInterval = 0;     // Track maximum interval (jitter analysis)
unsigned long minSampleInterval = 9999;  // Track minimum interval (jitter analysis)
unsigned long lastActualSampleTime = 0;  // For calculating actual intervals

// Calibration data storage
typedef struct {
  // Accelerometer calibration
  float accelBias[3];     // Offset correction (X, Y, Z)
  float accelScale[3];    // Scale correction (X, Y, Z)
  
  // Gyroscope calibration
  float gyroBias[3];      // Offset correction (X, Y, Z)
  float gyroScale[3];     // Scale correction (X, Y, Z)
  
  // Additional calibration data
  bool isCalibrated;      // Whether calibration has been performed
  unsigned long calibrationTimestamp; // When was calibration performed
} CalibrationData;

CalibrationData calibrationData = {
  {0.0f, 0.0f, 0.0f},  // accelBias
  {1.0f, 1.0f, 1.0f},  // accelScale
  {0.0f, 0.0f, 0.0f},  // gyroBias
  {1.0f, 1.0f, 1.0f},  // gyroScale
  false,               // isCalibrated
  0                    // calibrationTimestamp
};

// Feature extraction for authentication
typedef struct {
  float accMagnitudeMean;       // Mean acceleration magnitude
  float accMagnitudeStd;        // Standard deviation of acceleration magnitude
  float gyroMagnitudeMean;      // Mean gyroscope magnitude
  float gyroMagnitudeStd;       // Standard deviation of gyroscope magnitude
  float dominantFrequency;      // Dominant frequency of movement
  float energyRatio;            // Ratio of energy in specific frequency bands
} GestureFeatures;

GestureFeatures referenceFeatures;   // Features of enrolled gesture
GestureFeatures currentFeatures;     // Features of current authentication attempt

// Peak detection thresholds
#define PEAK_THRESHOLD_ACCEL 0.5  // Minimum acceleration change to detect a peak (in g)
#define PEAK_THRESHOLD_GYRO 50.0  // Minimum gyroscope change to detect a peak (in dps)

// Peak detection variables
float lastAccMag = 0;
float lastGyroMag = 0;
int accelPeakCount = 0;
int gyroPeakCount = 0;
float maxAccelPeak = 0;
float maxGyroPeak = 0;
bool wasAccelRising = false;
bool wasGyroRising = false;

// Correlation calculation variables
float accelCorrXY = 0;  // Correlation between X and Y acceleration
float accelCorrXZ = 0;  // Correlation between X and Z acceleration
float accelCorrYZ = 0;  // Correlation between Y and Z acceleration
float gyroCorrXY = 0;   // Correlation between X and Y rotation
float gyroCorrXZ = 0;   // Correlation between X and Z rotation
float gyroCorrYZ = 0;   // Correlation between Y and Z rotation

// Rolling window for correlation calculation
#define CORR_WINDOW_SIZE 50  // Number of samples to use for correlation
float accelXWindow[CORR_WINDOW_SIZE];
float accelYWindow[CORR_WINDOW_SIZE];
float accelZWindow[CORR_WINDOW_SIZE];
float gyroXWindow[CORR_WINDOW_SIZE];
float gyroYWindow[CORR_WINDOW_SIZE];
float gyroZWindow[CORR_WINDOW_SIZE];
int corrWindowIndex = 0;
bool corrWindowFull = false;

// Frequency band energy calculation
#define NUM_FREQ_BANDS 4  // Number of frequency bands to analyze
float accelFreqEnergy[NUM_FREQ_BANDS] = {0};  // Energy in each frequency band for accelerometer
float gyroFreqEnergy[NUM_FREQ_BANDS] = {0};   // Energy in each frequency band for gyroscope

// Rolling window for frequency analysis
#define FREQ_WINDOW_SIZE 64  // Power of 2 for efficient FFT
float accelFreqWindow[FREQ_WINDOW_SIZE];
float gyroFreqWindow[FREQ_WINDOW_SIZE];
int freqWindowIndex = 0;
bool freqWindowFull = false;

// Zero-crossing rate calculation
#define ZCR_WINDOW_SIZE 32  // Window size for ZCR calculation
float accelZCR[3] = {0};    // ZCR for each accelerometer axis
float gyroZCR[3] = {0};     // ZCR for each gyroscope axis

// Rolling windows for ZCR calculation
float accelXZCRWindow[ZCR_WINDOW_SIZE];
float accelYZCRWindow[ZCR_WINDOW_SIZE];
float accelZZCRWindow[ZCR_WINDOW_SIZE];
float gyroXZCRWindow[ZCR_WINDOW_SIZE];
float gyroYZCRWindow[ZCR_WINDOW_SIZE];
float gyroZZCRWindow[ZCR_WINDOW_SIZE];
int zcrWindowIndex = 0;
bool zcrWindowFull = false;

// Packet types for BLE transmission
#define PACKET_TYPE_SESSION_START 0x01
#define PACKET_TYPE_SENSOR_DATA   0x02
#define PACKET_TYPE_SESSION_END   0x03

// Binary data packet structure for BLE transmission (exactly 20 bytes)
typedef struct __attribute__((packed)) {
  uint8_t packetType;     // Packet type (START/DATA/END)
  uint8_t reserved;       // Reserved for future use (padding)
  uint16_t timestamp;     // Relative milliseconds since recording start (0-65535ms)
  uint16_t sampleId;      // Incremental sample number (0-65535)
  int16_t accX;          // Accelerometer X * 1000 (preserves 3 decimal places)
  int16_t accY;          // Accelerometer Y * 1000 
  int16_t accZ;          // Accelerometer Z * 1000
  int16_t gyroX;         // Gyroscope X * 10 (preserves 1 decimal place)
  int16_t gyroY;         // Gyroscope Y * 10
  int16_t gyroZ;         // Gyroscope Z * 10
  uint32_t recordingHash; // Hash of recording ID for packet identification
} BLEPacket;

// Global variables for BLE data transmission
uint16_t bleSampleCounter = 0;        // Counter for BLE sample IDs
uint32_t currentRecordingHash = 0;    // Hash of current recording ID

// Forward declarations
void processGestureData(GestureData* gesture);
void showRecordingProgress(unsigned long elapsedTime);
void updateProgressLED(unsigned long elapsedTime);
void calibrateIMU(bool fullCalibration);
float applyCalibration(float value, float bias, float scale);
void saveCalibrationToEEPROM();
void loadCalibrationFromEEPROM();
void extractFeatures(GestureData* gesture, GestureFeatures* features);
float calculateAuthenticationScore(GestureFeatures* ref, GestureFeatures* current);
void setAuthenticationMode(bool isAuth);
float calculateCorrelation(float* signal1, float* signal2, int size);
void updateCorrelations();
void calculateFreqBandEnergy(float* signal, float* energy, int windowSize);
float calculateZCR(float* signal, int windowSize);
void updateZCR();
void handleBLECommand(BLEDevice central, BLECharacteristic characteristic);
uint32_t hashString(const char* str);
void sendBLESensorData(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
void sendBLESessionStart();
void sendBLESessionEnd();

// Initialize BLE with proper service and characteristics
void initializeBLE() {
  if (!BLE.begin()) {
    Serial.println("ERROR: Starting BLE failed!");
    while(1);
  }
  
  // Set the device name that React Native will see
  BLE.setLocalName(BLE_DEVICE_NAME);
  
  // Set up the service
  BLE.setAdvertisedService(gestureService);
  
  // Add characteristics to the service
  gestureService.addCharacteristic(dataCharacteristic);
  gestureService.addCharacteristic(commandCharacteristic);
  
  // Add the service
  BLE.addService(gestureService);
  
  // Set up characteristic event handlers
  commandCharacteristic.setEventHandler(BLEWritten, handleBLECommand);
  
  // Set initial values for characteristics
  dataCharacteristic.writeValue(""); // Empty initial value
  commandCharacteristic.writeValue(""); // Empty initial value
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("BLE service initialized and advertising started:");
  Serial.print("Device Name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("MAC Address: ");
  Serial.println(BLE.address());
  Serial.print("Service UUID: ");
  Serial.println(BLE_SERVICE_UUID);
  Serial.print("Data Characteristic UUID: ");
  Serial.println(BLE_DATA_CHAR_UUID);
  Serial.print("Command Characteristic UUID: ");
  Serial.println(BLE_COMMAND_CHAR_UUID);
}

// Simple hash function for recording ID
uint32_t hashString(const char* str) {
  uint32_t hash = 5381;
  int c;
  while ((c = *str++)) {
    hash = ((hash << 5) + hash) + c; // hash * 33 + c
  }
  return hash;
}

// Send sensor data via BLE as binary packet
void sendBLESensorData(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  // Check if BLE is connected
  BLEDevice central = BLE.central();
  if (!central || !central.connected()) {
    return; // No connected device, skip BLE transmission
  }
  
  // Create binary data packet
  BLEPacket packet;
  
  // Fill packet data
  packet.packetType = PACKET_TYPE_SENSOR_DATA;
  packet.reserved = 0;
  packet.timestamp = (uint16_t)(millis() - recordingStartTime); // Relative timestamp
  packet.sampleId = bleSampleCounter++;
  
  // Scale and convert accelerometer data (preserve 3 decimal places)
  packet.accX = (int16_t)(accX * 1000.0f);
  packet.accY = (int16_t)(accY * 1000.0f);
  packet.accZ = (int16_t)(accZ * 1000.0f);
  
  // Scale and convert gyroscope data (preserve 1 decimal place)
  packet.gyroX = (int16_t)(gyroX * 10.0f);
  packet.gyroY = (int16_t)(gyroY * 10.0f);
  packet.gyroZ = (int16_t)(gyroZ * 10.0f);
  
  // Add recording ID hash
  packet.recordingHash = currentRecordingHash;
  
  // Send binary data via BLE notification
  dataCharacteristic.writeValue((uint8_t*)&packet, sizeof(BLEPacket));
  
  // Optional: Debug print packet info (can be removed for production)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 1000) { // Print debug info every second
    Serial.print("BLE: Sent sensor sample #");
    Serial.print(packet.sampleId);
    Serial.print(" to ");
    Serial.println(central.address());
    lastDebugPrint = millis();
  }
}

// Send session start notification via BLE
void sendBLESessionStart() {
  // Check if BLE is connected
  BLEDevice central = BLE.central();
  if (!central || !central.connected()) {
    return; // No connected device, skip BLE transmission
  }
  
  // Create session start packet
  BLEPacket packet;
  memset(&packet, 0, sizeof(BLEPacket)); // Clear all fields
  
  // Fill session start data
  packet.packetType = PACKET_TYPE_SESSION_START;
  packet.reserved = 0;
  packet.timestamp = 0; // Session start timestamp is always 0
  packet.sampleId = 0; // Reset sample counter
  packet.recordingHash = currentRecordingHash;
  
  // Send session start notification
  dataCharacteristic.writeValue((uint8_t*)&packet, sizeof(BLEPacket));
  
  Serial.print("BLE: Sent SESSION_START notification to ");
  Serial.print(central.address());
  Serial.print(" (Hash: 0x");
  Serial.print(currentRecordingHash, HEX);
  Serial.println(")");
}

// Send session end notification via BLE
void sendBLESessionEnd() {
  // Check if BLE is connected
  BLEDevice central = BLE.central();
  if (!central || !central.connected()) {
    return; // No connected device, skip BLE transmission
  }
  
  // Create session end packet
  BLEPacket packet;
  memset(&packet, 0, sizeof(BLEPacket)); // Clear all fields
  
  // Fill session end data
  packet.packetType = PACKET_TYPE_SESSION_END;
  packet.reserved = 0;
  packet.timestamp = (uint16_t)(millis() - recordingStartTime); // Total recording duration
  packet.sampleId = bleSampleCounter - 1; // Total samples sent
  packet.recordingHash = currentRecordingHash;
  
  // Send session end notification
  dataCharacteristic.writeValue((uint8_t*)&packet, sizeof(BLEPacket));
  
  Serial.print("BLE: Sent SESSION_END notification to ");
  Serial.print(central.address());
  Serial.print(" (Duration: ");
  Serial.print(packet.timestamp);
  Serial.print("ms, Samples: ");
  Serial.print(packet.sampleId + 1);
  Serial.println(")");
}

// Handle BLE command characteristic writes
void handleBLECommand(BLEDevice central, BLECharacteristic characteristic) {
  // Read the command data
  const uint8_t* data = characteristic.value();
  int length = characteristic.valueLength();
  
  if (length > 0) {
    Serial.print("BLE Command received from ");
    Serial.print(central.address());
    Serial.print(": ");
    
    // Convert to string for easier handling
    String command = "";
    for (int i = 0; i < length; i++) {
      command += (char)data[i];
    }
    
    Serial.println(command);
    
    // Handle different commands
    if (command == "ping") {
      Serial.println("Responding to ping command");
      // Could send a response via data characteristic if needed
    }
    else if (command == "status") {
      Serial.print("Status requested - Connected: ");
      Serial.println(central.connected() ? "Yes" : "No");
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
}

// Display device information for React Native app
void displayDeviceInfo() {
  Serial.println("\n=== DEVICE INFORMATION FOR REACT NATIVE ===");
  Serial.print("Device Name: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.print("Service UUID: ");
  Serial.println(BLE_SERVICE_UUID);
  Serial.print("Data Characteristic UUID: ");
  Serial.println(BLE_DATA_CHAR_UUID);
  Serial.print("Command Characteristic UUID: ");
  Serial.println(BLE_COMMAND_CHAR_UUID);
  Serial.println("============================================\n");
}

// Calculate the magnitude of acceleration vectors
float calculateAccMagnitude(float x, float y, float z) {
  return sqrt(x*x + y*y + z*z);
}

// Calculate the magnitude of rotation vectors
float calculateGyroMagnitude(float x, float y, float z) {
  return sqrt(x*x + y*y + z*z);
}

// Resample gesture to a fixed length for easier comparison
void resampleGesture(GestureData* gesture, float* outputAcc, float* outputGyro, int outputLength) {
  // Calculate magnitude at each sample point in original gesture
  float accMag[MAX_GESTURE_SAMPLES];
  float gyroMag[MAX_GESTURE_SAMPLES];
  
  for (int i = 0; i < gesture->sampleCount; i++) {
    accMag[i] = calculateAccMagnitude(
      gesture->accX[i], gesture->accY[i], gesture->accZ[i]);
    gyroMag[i] = calculateGyroMagnitude(
      gesture->gyroX[i], gesture->gyroY[i], gesture->gyroZ[i]);
  }
  
  // Resample to fixed length
  for (int j = 0; j < outputLength; j++) {
    // Calculate the corresponding index in the original series
    float srcIndex = (float)j * gesture->sampleCount / outputLength;
    int index1 = (int)srcIndex;
    int index2 = std::min(index1 + 1, gesture->sampleCount - 1);
    float fraction = srcIndex - index1;
    
    // Linear interpolation between samples
    outputAcc[j] = accMag[index1] * (1 - fraction) + accMag[index2] * fraction;
    outputGyro[j] = gyroMag[index1] * (1 - fraction) + gyroMag[index2] * fraction;
  }
}

// Turn off all LEDs
void ledOff() {
  digitalWrite(LED_RED, HIGH);    // HIGH = OFF (active LOW)
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

// Set RGB LED color (active LOW logic)
void setLEDColor(bool red, bool green, bool blue) {
  digitalWrite(LED_RED, !red);    // Invert because active LOW
  digitalWrite(LED_GREEN, !green);
  digitalWrite(LED_BLUE, !blue);
}

// Decode TAP_SRC register for debugging
void decodeTapSource(uint8_t tapSource) {
  Serial.print("TAP_SRC = 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print((tapSource >> i) & 0x01);
  }
  Serial.print(" (0x");
  Serial.print(tapSource, HEX);
  Serial.println(")");
  
  // Bit-by-bit analysis
  Serial.println("Bit analysis:");
  if (tapSource & 0x80) Serial.println("  Bit 7: TAP_IA - Tap event detected");
  if (tapSource & 0x40) Serial.println("  Bit 6: TAP_EVENT - Single/double tap event completed");
  if (tapSource & 0x20) Serial.println("  Bit 5: TAP_SIGN - Sign of acceleration peak");
  if (tapSource & 0x10) Serial.println("  Bit 4: DOUBLE_TAP - Double tap detected");
  if (tapSource & 0x08) Serial.println("  Bit 3: SINGLE_TAP - Single tap detected");
  if (tapSource & 0x04) Serial.println("  Bit 2: Z_TAP - Z-axis tap detected");
  if (tapSource & 0x02) Serial.println("  Bit 1: Y_TAP - Y-axis tap detected");
  if (tapSource & 0x01) Serial.println("  Bit 0: X_TAP - X-axis tap detected");
  
  // Timing information
  unsigned long currentTime = millis();
  if (lastTapSource != 0) {
    Serial.print("Time since last tap activity: ");
    Serial.print(currentTime - lastNonZeroReadTime);
    Serial.println(" ms");
  }
  
  // Start or update tap sequence timing
  if (!tapSequenceActive && (tapSource != 0)) {
    tapSequenceActive = true;
    tapSequenceStartTime = currentTime;
    tapCount = 1;
    Serial.println("Starting new tap sequence");
  } else if (tapSequenceActive && (tapSource != 0)) {
    // Count every tap activity, even if the same source value
    tapCount++;
    Serial.print("Tap #");
    Serial.print(tapCount);
    Serial.print(" in sequence, ");
    Serial.print(currentTime - tapSequenceStartTime);
    Serial.println(" ms since sequence start");
  }
  
  // Reset tap sequence if too much time has passed
  if (tapSequenceActive && (currentTime - lastNonZeroReadTime > 700)) {
    Serial.println("Tap sequence timed out, resetting");
    tapSequenceActive = false;
    // Print CSV header here so it appears right before data
    Serial.println("rel_timestamp,recording_id,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
  }
  
  lastNonZeroReadTime = currentTime;
  lastTapSource = tapSource;
}

// Configure tap detection
void configureTapDetection() {
  // Reset the device first to ensure clean configuration
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x01);  // Reset bit
  delay(10);  // Wait for reset
  
  // Enable embedded functions
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C); // Enable gyro/accel axes and embedded functions
  
  // Set accelerometer ODR to 416 Hz and range to 2g
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);  // 0x60 = 416Hz, ±2g
  
  // Enable gyroscope with 416 Hz ODR and 2000 dps
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x60);  // 0x60 = 416Hz, 2000dps
  
  // TAP_CFG (0x58): Enable X, Y, Z tap detection and LIR (Latched Interrupt)
  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8F);  // 0x8F = Enable XYZ + LIR
  
  // Set tap threshold
  // TAP_THS_6D (0x59): Threshold value
  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x08);  // 0x08 (less sensitive)
  
  // Configure tap recognition parameters
  // INT_DUR2 (0x5A): Values optimized based on user's tapping pattern
  // bits[7:4]=Duration, bits[3:2]=Quiet, bits[1:0]=Shock
  imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x26);  // 0x26 = Duration=2, Quiet=1, Shock=2
  
  // Configure for double tap detection
  // WAKE_UP_THS (0x5B): Bit 7=0 for double tap mode
  imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x00);  // Bit 7=0 for double tap mode
  
  // Read back configuration registers to verify
  uint8_t tapCfg = 0, tapThs = 0, intDur2 = 0, wakeUpThs = 0, ctrl10 = 0, ctrl1 = 0;
  
  imu.readRegister(&ctrl1, LSM6DS3_ACC_GYRO_CTRL1_XL);
  imu.readRegister(&ctrl10, LSM6DS3_ACC_GYRO_CTRL10_C);
  imu.readRegister(&tapCfg, LSM6DS3_ACC_GYRO_TAP_CFG1);
  imu.readRegister(&tapThs, LSM6DS3_ACC_GYRO_TAP_THS_6D);
  imu.readRegister(&intDur2, LSM6DS3_ACC_GYRO_INT_DUR2);
  imu.readRegister(&wakeUpThs, LSM6DS3_ACC_GYRO_WAKE_UP_THS);
  
  Serial.println("Double Tap Detection Configuration:");
  Serial.print("CTRL1_XL: 0x"); Serial.println(ctrl1, HEX);
  Serial.print("CTRL10_C: 0x"); Serial.println(ctrl10, HEX);
  Serial.print("TAP_CFG: 0x"); Serial.println(tapCfg, HEX);
  Serial.print("TAP_THS: 0x"); Serial.println(tapThs, HEX);
  Serial.print("INT_DUR2: 0x"); Serial.println(intDur2, HEX);
  Serial.print("WAKE_UP_THS: 0x"); Serial.println(wakeUpThs, HEX);
  
  Serial.print("Software double tap window: "); 
  Serial.print(DOUBLE_TAP_WINDOW);
  Serial.println(" ms");
}

// Generate a unique recording ID based on time and a random number
void generateRecordingId() {
  // Use current time and a random value to create a unique ID
  sprintf(recordingId, "g_%lu_%ld", millis(), random(1000, 9999));
}

// Print metadata about the recording session
void printRecordingMetadata() {
  Serial.println("\n------ RECORDING METADATA ------");
  Serial.print("Recording duration: ");
  Serial.print(RECORDING_DURATION);
  Serial.println(" ms");
  
  Serial.print("Sample rate: ");
  Serial.print(1000 / SAMPLE_RATE_MS);
  Serial.println(" Hz");
  
  Serial.println("\nSensor configuration:");
  Serial.println("- IMU: LSM6DS3 (Accelerometer + Gyroscope)");
  
  // Read configuration registers for metadata
  uint8_t ctrl1, ctrl2;
  imu.readRegister(&ctrl1, LSM6DS3_ACC_GYRO_CTRL1_XL);
  imu.readRegister(&ctrl2, LSM6DS3_ACC_GYRO_CTRL2_G);
  
  // Determine accelerometer range
  String accRange;
  switch(ctrl1 & 0x0C) {
    case 0x00: accRange = "±2g"; break;
    case 0x04: accRange = "±4g"; break;
    case 0x08: accRange = "±8g"; break;
    case 0x0C: accRange = "±16g"; break;
  }
  
  // Determine gyroscope range
  String gyroRange;
  switch(ctrl2 & 0x0C) {
    case 0x00: gyroRange = "250 dps"; break;
    case 0x04: gyroRange = "500 dps"; break;
    case 0x08: gyroRange = "1000 dps"; break;
    case 0x0C: gyroRange = "2000 dps"; break;
  }
  
  Serial.print("- Accelerometer range: "); Serial.println(accRange);
  Serial.print("- Gyroscope range: "); Serial.println(gyroRange);
  
  Serial.println("\nSampling Considerations:");
  Serial.print("* Sample rate: "); 
  Serial.print(1000 / SAMPLE_RATE_MS);
  Serial.println("Hz provides detailed data for gesture recognition");
  Serial.print("* Duration: Recording for approximately ");
  Serial.print(RECORDING_DURATION / 1000);
  Serial.println(" seconds per gesture");
  
  if (calibrationData.isCalibrated) {
    Serial.println("* Calibration: Sensor calibration applied");
  } else {
    Serial.println("* Calibration: Using raw sensor values");
  }
  
  Serial.println("------------------------------");
}

// Replace showRecordingProgress with updateProgressLED that only changes the LED color
void updateProgressLED(unsigned long elapsedTime) {
  // Calculate progress percentage
  int progressPercent = (elapsedTime * 100) / RECORDING_DURATION;
  progressPercent = constrain(progressPercent, 0, 100);
  
  // Change LED color intensity based on progress without printing to serial
  if (progressPercent < 50) {
    // Fade from green to yellow as we approach 50%
    int redIntensity = map(progressPercent, 0, 50, 0, 1);
    setLEDColor(redIntensity, true, false);
  } else {
    // Fade from yellow to red as we approach 100%
    int greenIntensity = map(progressPercent, 50, 100, 1, 0);
    setLEDColor(true, greenIntensity, false);
  }
}

// Add the progress indicator function before recording starts
void showRecordingProgress(unsigned long elapsedTime) {
  // Calculate progress percentage
  int progressPercent = (elapsedTime * 100) / RECORDING_DURATION;
  progressPercent = constrain(progressPercent, 0, 100);
  
  // Display progress bar in serial monitor
  Serial.print("\rRecording: [");
  
  // Print progress bar
  for (int i = 0; i < 20; i++) {
    if (i < progressPercent / 5) {
      Serial.print("=");
    } else {
      Serial.print(" ");
    }
  }
  
  Serial.print("] ");
  Serial.print(progressPercent);
  Serial.print("% (");
  Serial.print(elapsedTime / 1000);
  Serial.print(".0s / ");
  Serial.print(RECORDING_DURATION / 1000);
  Serial.print(".0s)");
}

// Process recorded gesture data for authentication
void processGestureData(GestureData* gesture) {
  // Ensure valid data
  if (gesture->sampleCount < 10) {
    Serial.println("Warning: Gesture too short, may be invalid");
    return;
  }
  
  Serial.print("Processing gesture data (");
  Serial.print(gesture->sampleCount);
  Serial.println(" samples)");
  
  // Mark the gesture as active (valid)
  gesture->isActive = true;
  
  // Extract features for authentication
  if (authenticationMode) {
    // Extract features from current gesture
    extractFeatures(gesture, &currentFeatures);
    
    // Calculate similarity with reference gestures
    float bestScore = 0;
    int bestMatchIndex = -1;
    
    // Find best matching reference gesture
    for (int i = 0; i < 3; i++) {
      if (referenceGestures[i].isActive) {
        // Extract features from reference if not done yet
        GestureFeatures refFeatures;
        extractFeatures(&referenceGestures[i], &refFeatures);
        
        // Calculate authentication score
        float score = calculateAuthenticationScore(&refFeatures, &currentFeatures);
        
        Serial.print("Authentication score with reference #");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.println(score, 4);
        
        if (score > bestScore) {
          bestScore = score;
          bestMatchIndex = i;
        }
      }
    }
    
    // Authentication result
    authAttemptsCount++;
    Serial.println("\n*** AUTHENTICATION RESULT ***");
    Serial.print("Best match: Reference gesture #");
    Serial.println(bestMatchIndex + 1);
    Serial.print("Score: ");
    Serial.print(bestScore * 100);
    Serial.println("%");
    
    if (bestScore >= AUTH_THRESHOLD) {
      Serial.println("AUTHENTICATION SUCCESSFUL!");
      
      // Flash green LED to indicate success
      for (int i = 0; i < 5; i++) {
        setLEDColor(false, true, false); // Green
        delay(100);
        ledOff();
        delay(100);
      }
    } else {
      Serial.println("AUTHENTICATION FAILED!");
      
      // Flash red LED to indicate failure
      for (int i = 0; i < 5; i++) {
        setLEDColor(true, false, false); // Red
        delay(100);
        ledOff();
        delay(100);
      }
    }
    
    // Return to authentication mode
    setLEDColor(false, false, true); // Blue
  } else {
    // In enrollment mode, just extract and save features if this is the last reference
    if (currentReferenceIndex >= 2) {
      extractFeatures(gesture, &referenceFeatures);
      Serial.println("Reference features extracted and saved for authentication");
    }
  }
  
  Serial.println("Gesture processing complete");
}

// Function to detect peaks in a signal
bool detectPeak(float current, float last, float threshold, bool wasRising) {
    bool isRising = current > last;
    bool peakDetected = wasRising && !isRising && (abs(current - last) > threshold);
    return peakDetected;
}

// Calculate correlation coefficient between two signals
float calculateCorrelation(float* signal1, float* signal2, int size) {
    float sum1 = 0, sum2 = 0, sum1Sq = 0, sum2Sq = 0, pSum = 0;
    
    for (int i = 0; i < size; i++) {
        sum1 += signal1[i];
        sum2 += signal2[i];
        sum1Sq += signal1[i] * signal1[i];
        sum2Sq += signal2[i] * signal2[i];
        pSum += signal1[i] * signal2[i];
    }
    
    float num = pSum - (sum1 * sum2 / size);
    float den = sqrt((sum1Sq - sum1 * sum1 / size) * (sum2Sq - sum2 * sum2 / size));
    
    if (den == 0) return 0;
    return num / den;
}

// Update correlation calculations
void updateCorrelations() {
    if (!corrWindowFull) return;
    
    // Calculate accelerometer correlations
    accelCorrXY = calculateCorrelation(accelXWindow, accelYWindow, CORR_WINDOW_SIZE);
    accelCorrXZ = calculateCorrelation(accelXWindow, accelZWindow, CORR_WINDOW_SIZE);
    accelCorrYZ = calculateCorrelation(accelYWindow, accelZWindow, CORR_WINDOW_SIZE);
    
    // Calculate gyroscope correlations
    gyroCorrXY = calculateCorrelation(gyroXWindow, gyroYWindow, CORR_WINDOW_SIZE);
    gyroCorrXZ = calculateCorrelation(gyroXWindow, gyroZWindow, CORR_WINDOW_SIZE);
    gyroCorrYZ = calculateCorrelation(gyroYWindow, gyroZWindow, CORR_WINDOW_SIZE);
}

// Calculate energy in frequency bands using a simple moving average filter
void calculateFreqBandEnergy(float* signal, float* energy, int windowSize) {
    // Simple band-pass filtering using moving averages of different lengths
    // This approximates frequency bands without full FFT
    float band1 = 0, band2 = 0, band3 = 0, band4 = 0;
    
    // Band 1: Very low frequency (0-2 Hz) - long moving average
    for (int i = 0; i < windowSize; i++) {
        band1 += signal[i];
    }
    band1 /= windowSize;
    
    // Band 2: Low frequency (2-5 Hz) - medium moving average
    for (int i = 0; i < windowSize/2; i++) {
        band2 += signal[i];
    }
    band2 = (band2 / (windowSize/2)) - band1;
    
    // Band 3: Medium frequency (5-10 Hz) - short moving average
    for (int i = 0; i < windowSize/4; i++) {
        band3 += signal[i];
    }
    band3 = (band3 / (windowSize/4)) - band2 - band1;
    
    // Band 4: High frequency (>10 Hz) - very short moving average
    for (int i = 0; i < windowSize/8; i++) {
        band4 += signal[i];
    }
    band4 = (band4 / (windowSize/8)) - band3 - band2 - band1;
    
    // Store energy values (squared to get power)
    energy[0] = band1 * band1;
    energy[1] = band2 * band2;
    energy[2] = band3 * band3;
    energy[3] = band4 * band4;
}

// Calculate zero-crossing rate for a signal window
float calculateZCR(float* signal, int windowSize) {
    int crossings = 0;
    for (int i = 1; i < windowSize; i++) {
        // Count sign changes, ignoring small fluctuations near zero
        if ((signal[i] > 0.01f && signal[i-1] < -0.01f) || 
            (signal[i] < -0.01f && signal[i-1] > 0.01f)) {
            crossings++;
        }
    }
    return (float)crossings / (windowSize - 1);  // Normalize by window size
}

// Update zero-crossing rates for all axes
void updateZCR() {
    if (!zcrWindowFull) return;
    
    // Calculate ZCR for accelerometer axes
    accelZCR[0] = calculateZCR(accelXZCRWindow, ZCR_WINDOW_SIZE);
    accelZCR[1] = calculateZCR(accelYZCRWindow, ZCR_WINDOW_SIZE);
    accelZCR[2] = calculateZCR(accelZZCRWindow, ZCR_WINDOW_SIZE);
    
    // Calculate ZCR for gyroscope axes
    gyroZCR[0] = calculateZCR(gyroXZCRWindow, ZCR_WINDOW_SIZE);
    gyroZCR[1] = calculateZCR(gyroYZCRWindow, ZCR_WINDOW_SIZE);
    gyroZCR[2] = calculateZCR(gyroZZCRWindow, ZCR_WINDOW_SIZE);
}

// Record and print current sensor data
void recordSensorData() {
  // Read sensor data
  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  float gyroX = imu.readFloatGyroX();
  float gyroY = imu.readFloatGyroY();
  float gyroZ = imu.readFloatGyroZ();
  
  // Apply calibration if available
  if (calibrationData.isCalibrated) {
    // Apply accelerometer calibration
    accX = (accX - calibrationData.accelBias[0]) * calibrationData.accelScale[0];
    accY = (accY - calibrationData.accelBias[1]) * calibrationData.accelScale[1];
    accZ = (accZ - calibrationData.accelBias[2]) * calibrationData.accelScale[2];
    
    // Apply gyroscope calibration
    gyroX -= calibrationData.gyroBias[0];
    gyroY -= calibrationData.gyroBias[1];
    gyroZ -= calibrationData.gyroBias[2];
  }
  
  // Store data in current gesture
  if (currentGesture.sampleCount < MAX_GESTURE_SAMPLES) {
    currentGesture.accX[currentGesture.sampleCount] = accX;
    currentGesture.accY[currentGesture.sampleCount] = accY;
    currentGesture.accZ[currentGesture.sampleCount] = accZ;
    currentGesture.gyroX[currentGesture.sampleCount] = gyroX;
    currentGesture.gyroY[currentGesture.sampleCount] = gyroY;
    currentGesture.gyroZ[currentGesture.sampleCount] = gyroZ;
    currentGesture.sampleCount++;
  }
  
  // Send data via BLE (binary format)
  sendBLESensorData(accX, accY, accZ, gyroX, gyroY, gyroZ);
  
  // Print data in CSV format for debugging (kept for development)
  Serial.print(millis() - recordingStartTime); Serial.print(",");
  Serial.print(recordingId); Serial.print(",");
  Serial.print(accX, 6); Serial.print(",");
  Serial.print(accY, 6); Serial.print(",");
  Serial.print(accZ, 6); Serial.print(",");
  Serial.print(gyroX, 6); Serial.print(",");
  Serial.print(gyroY, 6); Serial.print(",");
  Serial.println(gyroZ, 6);
}

// Calculate and print features after recording is complete
void calculateAndPrintFeatures() {
  if (currentGesture.sampleCount == 0) {
    Serial.println("No data to analyze");
    return;
  }

  Serial.println("\n=== FEATURE ANALYSIS ===");
  // Extract features from the recorded gesture
  GestureFeatures currentFeatures;
  extractFeatures(&currentGesture, &currentFeatures);
  
  // Print feature analysis summary
  Serial.println("Feature analysis complete");
  Serial.print("Samples analyzed: ");
  Serial.println(currentGesture.sampleCount);
}

// Handle double tap detection - now manages recording window
void handleDoubleTap() {
  Serial.println("*** DOUBLE TAP DETECTED! ***");

  // Set flag to ignore taps during gesture recording
  ignoreTaps = true;
  
  // Flash LED to indicate detected double tap
  for (int i = 0; i < 2; i++) {
    ledOff();
    delay(75);
    setLEDColor(true, true, true); // White flash
    delay(75);
  }
  ledOff();
  
  // Generate new recording ID and set absolute start time
  generateRecordingId();
  absoluteStartTime = millis();
  
  // Initialize BLE transmission variables
  bleSampleCounter = 0;
  currentRecordingHash = hashString(recordingId);
  
  // Add delay before starting recording to avoid accidental motion
  Serial.println("Preparing to record in 350ms...");
  delay(350);  // 350ms pause before starting recording
  
  if (authenticationMode) {
    // Authentication mode
    Serial.println("Recording AUTHENTICATION gesture");
  } else {
    // We're in enrollment mode
    Serial.print("Recording ENROLLMENT gesture #");
    Serial.println(currentReferenceIndex + 1);
  }
  
  // Start recording immediately
  Serial.println("NOW BEGIN YOUR GESTURE!");
  Serial.println("Recording started");
  
  if (authenticationMode) {
    setLEDColor(false, false, true); // Blue - authentication mode
  } else {
    setLEDColor(false, true, false); // Green - enrollment mode
  }
  
  // Start recording window
  isRecording = true;
  recordingStartTime = millis();
  sampleCount = 0;
  
  // Initialize timing variables for consistent sampling
  nextSampleTime = recordingStartTime;
  lastActualSampleTime = recordingStartTime;
  actualSampleInterval = 0;
  maxSampleInterval = 0;
  minSampleInterval = 9999;
  
  // Reset current gesture data
  currentGesture.isActive = true;
  currentGesture.sampleCount = 0;
  currentGesture.gestureID = currentReferenceIndex;
  currentGesture.repetition = 1;
  
  // Reset peak detection variables
  lastAccMag = 0;
  lastGyroMag = 0;
  accelPeakCount = 0;
  gyroPeakCount = 0;
  maxAccelPeak = 0;
  maxGyroPeak = 0;
  wasAccelRising = false;
  wasGyroRising = false;
  
  // Reset correlation variables
  corrWindowIndex = 0;
  corrWindowFull = false;
  accelCorrXY = 0;
  accelCorrXZ = 0;
  accelCorrYZ = 0;
  gyroCorrXY = 0;
  gyroCorrXZ = 0;
  gyroCorrYZ = 0;
  
  // Reset frequency analysis variables
  freqWindowIndex = 0;
  freqWindowFull = false;
  for (int i = 0; i < NUM_FREQ_BANDS; i++) {
    accelFreqEnergy[i] = 0;
    gyroFreqEnergy[i] = 0;
  }
  
  // Reset ZCR variables
  zcrWindowIndex = 0;
  zcrWindowFull = false;
  for (int i = 0; i < 3; i++) {
    accelZCR[i] = 0;
    gyroZCR[i] = 0;
  }
  
  // Send session start notification via BLE
  sendBLESessionStart();
  
  // Print metadata and CSV header for the data
  printRecordingMetadata();
  Serial.println("\nRECORDING STARTED - 4 second window");
  
  // Print BLE packet information
  Serial.println("\nBLE Binary Packet Format (20 bytes):");
  Serial.print("Recording Hash: 0x");
  Serial.println(currentRecordingHash, HEX);
  Serial.println("Packet Structure: [type][reserved][timestamp][sampleId][accX][accY][accZ][gyroX][gyroY][gyroZ][hash]");
  Serial.println("Packet Types: START=0x01, DATA=0x02, END=0x03");
  Serial.println("Data Scaling: Accel x1000 (±32.767g), Gyro x10 (±3276.7dps)");
  
  // Print CSV header here so it appears right before data
  Serial.println("\nCSV Debug Output:");
  Serial.println("rel_timestamp,recording_id,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  // Initialize random number generator with a somewhat random seed
  randomSeed(analogRead(0));
  
  Serial.println("XIAO nRF52840 Sense - Gesture Recording");
  Serial.println("----------------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'reset' - Reset to reference gesture recording mode");
  Serial.println("  'calibrate' - Run IMU calibration (place device on flat surface)");
  Serial.println("  'fullcal' - Run extended IMU calibration (follow instructions)");
  Serial.println("----------------------------------------------");
  Serial.println("This program will record 3 reference gestures");
  Serial.println("Double tap to start recording each gesture");
  Serial.println("----------------------------------------------");
  
  // Initialize RGB LED pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledOff();
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize LSM6DS3 sensor
  if (imu.begin() != 0) {
    Serial.println("ERROR: Failed to initialize LSM6DS3!");
    setLEDColor(true, false, false);  // Red LED indicates error
    while(1);
  }
  
  Serial.println("LSM6DS3 initialized successfully");
  
  // Initialize BLE for React Native discovery
  initializeBLE();
  
  // Display device information for React Native app
  displayDeviceInfo();
  
  // Run basic calibration on startup
  Serial.println("Performing initial calibration...");
  Serial.println("Please place device on a flat surface and keep it still.");
  delay(2000); // Give user time to position the device
  calibrateIMU(false); // Run basic calibration
  
  configureTapDetection();
  
  // Initialize gesture data structures
  for (int i = 0; i < 3; i++) {
    referenceGestures[i].isActive = false;
    referenceGestures[i].sampleCount = 0;
  }
  currentGesture.isActive = false;
  currentGesture.sampleCount = 0;
  
  Serial.println("Ready to record reference gestures");
  Serial.println("Double tap to start recording reference gesture #1");
  
  // Set initial LED color to RED (waiting mode)
  setLEDColor(true, false, false);
}

void loop() {
  // Poll BLE events and handle connections
  BLE.poll();
  
  // Check for BLE connection events
  BLEDevice central = BLE.central();
  static bool wasConnected = false;
  bool isConnected = central && central.connected();
  
  // Handle connection state changes
  if (isConnected && !wasConnected) {
    Serial.print("BLE Connected to: ");
    Serial.println(central.address());
    // Flash blue LED to indicate BLE connection
    for (int i = 0; i < 2; i++) {
      setLEDColor(false, false, true); // Blue
      delay(100);
      ledOff();
      delay(100);
    }
  }
  else if (!isConnected && wasConnected) {
    Serial.println("BLE Disconnected");
    // Flash red LED to indicate disconnection
    for (int i = 0; i < 2; i++) {
      setLEDColor(true, false, false); // Red
      delay(100);
      ledOff();
      delay(100);
    }
  }
  
  wasConnected = isConnected;
  
  // Check if we should record sensor data during recording window
  if (isRecording) {
    unsigned long currentTime = millis();
    
    // Improved timing: check if it's time for the next scheduled sample
    if (currentTime >= nextSampleTime) {
      // Record sample and update timing info
      recordSensorData();
      
      // Calculate actual interval for this sample
      if (lastActualSampleTime > 0) {
        actualSampleInterval = currentTime - lastActualSampleTime;
        // Track timing statistics to assess jitter
        if (actualSampleInterval > maxSampleInterval) {
          maxSampleInterval = actualSampleInterval;
        }
        if (actualSampleInterval < minSampleInterval) {
          minSampleInterval = actualSampleInterval;
        }
      }
      lastActualSampleTime = currentTime;
      
      // Schedule next sample exactly sampleInterval ms from the theoretical nextSampleTime
      // This prevents drift from cumulative timing errors
      nextSampleTime += sampleInterval;
      
      // Show recording progress every 500ms
      unsigned long elapsedTime = currentTime - recordingStartTime;
      if (elapsedTime % 500 < sampleInterval) {
        // Use LED color for progress indication only, don't print to serial during CSV recording
        updateProgressLED(elapsedTime);
      }
    }
    
    // Check if recording window has ended
    if (currentTime - recordingStartTime >= RECORDING_DURATION) {
      // Recording window ended
      isRecording = false;
      
      // Send session end notification via BLE
      sendBLESessionEnd();
      
      // Update status LED to indicate recording is complete
      setLEDColor(false, true, false); // Green - recording complete
      
      // Log recording completion
      Serial.println();  // Add a newline to separate from CSV data
      Serial.println("\nRECORDING FINISHED - " + String(currentGesture.sampleCount) + " samples collected");
      
      // Process the recorded data
      processGestureData(&currentGesture);
      
      // Regular reference gesture recording
      // Store this as a reference gesture
      memcpy(&referenceGestures[currentReferenceIndex], &currentGesture, sizeof(GestureData));
      referenceGestures[currentReferenceIndex].isActive = true;
      
      // Move to next reference or finish recording
      currentReferenceIndex++;
      if (currentReferenceIndex >= 3) {
        // We've recorded all references
        currentReferenceIndex = 0;
        Serial.println("\n*** All reference gestures recorded! ***");
        Serial.println("You can now use the 'reset' command to start over if needed.");
        
        // Celebration light pattern - all gestures recorded
        for (int i = 0; i < 3; i++) {
          // Red
          setLEDColor(true, false, false);
          delay(150);
          // Green
          setLEDColor(false, true, false);
          delay(150);
          // Blue
          setLEDColor(false, false, true);
          delay(150);
        }
        
        // Set LED to yellow to indicate all recordings complete
        setLEDColor(true, true, false);
      } else {
        // More references to record
        Serial.print("\nReady for next reference gesture. Double-tap to record reference #");
        Serial.println(currentReferenceIndex + 1);
        
        // Return to waiting state (red LED)
        setLEDColor(true, false, false);
      }
      
      // Allow double taps again
      ignoreTaps = false;
    }
    
    // During recording, we only focus on collecting data, not detecting taps
    return;
  }
  
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "reset") {
      // Reset to reference recording mode
      currentReferenceIndex = 0;
      
      // Reset reference gestures
      for (int i = 0; i < 3; i++) {
        referenceGestures[i].isActive = false;
        referenceGestures[i].sampleCount = 0;
      }
      
      Serial.println("\n*** RESET TO ENROLLMENT MODE ***");
      Serial.println("Double tap to record enrollment gesture #1");
      
      // Set to enrollment mode
      setAuthenticationMode(false);
      
      // Blink red LED to indicate reset
      for (int i = 0; i < 3; i++) {
        setLEDColor(true, false, false); // Red
        delay(200);
        ledOff();
        delay(200);
      }
      setLEDColor(true, false, false); // Back to red (waiting state)
    }
    else if (command == "enroll") {
      // Switch to enrollment mode
      currentReferenceIndex = 0;
      
      // Reset reference gestures
      for (int i = 0; i < 3; i++) {
        referenceGestures[i].isActive = false;
        referenceGestures[i].sampleCount = 0;
      }
      
      // Set to enrollment mode
      setAuthenticationMode(false);
      
      Serial.println("\n*** ENROLLMENT MODE ACTIVATED ***");
      Serial.println("Double tap to record enrollment gesture #1");
    }
    else if (command == "auth") {
      // Switch to authentication mode
      setAuthenticationMode(true);
    }
    else if (command == "stats") {
      // Show authentication statistics
      Serial.println("\n*** AUTHENTICATION STATISTICS ***");
      
      // Count active reference gestures
      int activeCount = 0;
      for (int i = 0; i < 3; i++) {
        if (referenceGestures[i].isActive) activeCount++;
      }
      
      Serial.print("Reference gestures: ");
      Serial.print(activeCount);
      Serial.println("/3");
      
      Serial.print("Authentication attempts: ");
      Serial.println(authAttemptsCount);
      
      Serial.print("Current mode: ");
      Serial.println(authenticationMode ? "AUTHENTICATION" : "ENROLLMENT");
      
      // Show features of reference gestures if available
      if (activeCount > 0) {
        Serial.println("\nReference gesture features:");
        for (int i = 0; i < 3; i++) {
          if (referenceGestures[i].isActive) {
            Serial.print("Gesture #");
            Serial.print(i+1);
            Serial.print(" (");
            Serial.print(referenceGestures[i].sampleCount);
            Serial.println(" samples):");
            
            GestureFeatures features;
            extractFeatures(&referenceGestures[i], &features);
            
            Serial.print("  Acc Mean: "); 
            Serial.print(features.accMagnitudeMean, 2);
            Serial.print("g, Std: ");
            Serial.println(features.accMagnitudeStd, 2);
            
            Serial.print("  Gyro Mean: "); 
            Serial.print(features.gyroMagnitudeMean, 2);
            Serial.print("dps, Std: ");
            Serial.println(features.gyroMagnitudeStd, 2);
            
            Serial.print("  Dom. Frequency: ");
            Serial.print(features.dominantFrequency, 2);
            Serial.print("Hz, Energy Ratio: ");
            Serial.println(features.energyRatio, 2);
          }
        }
      }
    }
    else if (command == "calibrate") {
      // Run basic calibration (single orientation)
      Serial.println("\n*** STARTING BASIC CALIBRATION ***");
      Serial.println("Place the device on a level surface and keep it still");
      
      // Brief pause to let user position the device
      setLEDColor(false, false, true); // Blue
      delay(3000);
      
      // Run the calibration
      calibrateIMU(false);
      
      Serial.println("Calibration complete - back to recording mode");
      Serial.println("Double tap to record a gesture");
      
      // Back to waiting state
      setLEDColor(authenticationMode ? false : true, false, authenticationMode ? true : false); // Red or blue based on mode
    }
    else if (command == "fullcal") {
      // Run full calibration (multi-orientation)
      Serial.println("\n*** STARTING FULL CALIBRATION ***");
      Serial.println("Follow the instructions for multi-orientation calibration");
      
      // Indicate extended calibration mode
      for (int i = 0; i < 5; i++) {
        setLEDColor(false, false, true); // Blue
        delay(100);
        ledOff();
        delay(100);
      }
      
      // Run the full calibration
      calibrateIMU(true);
      
      Serial.println("Full calibration complete - back to recording mode");
      Serial.println("Double tap to record a gesture");
      
      // Back to waiting state
      setLEDColor(authenticationMode ? false : true, false, authenticationMode ? true : false); // Red or blue based on mode
    }
    else if (command == "info") {
      // Display system information and current calibration
      Serial.println("\n*** SYSTEM INFORMATION ***");
      Serial.print("Device ID: ");
      Serial.println(DEVICE_ID);
      Serial.print("Firmware version: 1.2.0");
      Serial.println(" (with authentication)");
      
      // Calibration status
      Serial.println("\nCalibration Status:");
      if (calibrationData.isCalibrated) {
        Serial.println("  Calibrated: YES");
        
        // Calculate time since calibration
        unsigned long calibAge = millis() - calibrationData.calibrationTimestamp;
        Serial.print("  Calibration age: ");
        if (calibAge < 60000) {
          Serial.print(calibAge / 1000);
          Serial.println(" seconds ago");
        } else if (calibAge < 3600000) {
          Serial.print(calibAge / 60000);
          Serial.println(" minutes ago");
        } else {
          Serial.print(calibAge / 3600000);
          Serial.println(" hours ago");
        }
        
        // Display gravity vector magnitude
        float gravityMag = sqrt(
          pow(1.0 - calibrationData.accelBias[0], 2) +
          pow(0.0 - calibrationData.accelBias[1], 2) +
          pow(0.0 - calibrationData.accelBias[2], 2)
        );
        Serial.print("  Gravity magnitude: ");
        Serial.print(gravityMag, 4);
        Serial.println("g (should be close to 1.0)");
      } else {
        Serial.println("  Calibrated: NO (using raw sensor data)");
        Serial.println("  Run 'calibrate' command to calibrate the sensor");
      }
      
      Serial.println("\nAuthentication Status:");
      Serial.print("  Mode: ");
      Serial.println(authenticationMode ? "AUTHENTICATION" : "ENROLLMENT");
      Serial.print("  Reference gestures recorded: ");
      int count = 0;
      for (int i = 0; i < 3; i++) {
        if (referenceGestures[i].isActive) count++;
      }
      Serial.print(count);
      Serial.println("/3");
      Serial.print("  Authentication attempts: ");
      Serial.println(authAttemptsCount);
      Serial.print("  Authentication threshold: ");
      Serial.print(AUTH_THRESHOLD * 100);
      Serial.println("%");
      
      Serial.println("\nCommands:");
      Serial.println("  'reset' - Reset to initial state");
      Serial.println("  'enroll' - Enter enrollment mode");
      Serial.println("  'auth' - Enter authentication mode");
      Serial.println("  'stats' - Show authentication statistics");
      Serial.println("  'calibrate' - Calibrate sensors");
      Serial.println("  'info' - Show this information");
    }
  }
  
  // Skip tap detection if we're in another mode that should ignore taps
  if (ignoreTaps) {
    delay(10);
    return;
  }
  
  // Read the TAP_SRC register to check for tap events
  uint8_t tapSource = 0;
  imu.readRegister(&tapSource, LSM6DS3_ACC_GYRO_TAP_SRC);
  
  // Check for hardware double tap event - look for double tap bit (0x10)
  if (tapSource & 0x10) {
    handleDoubleTap();
  }
  // Also check for single tap events for software double tap detection (bit 3 = 0x08) 
  else if (tapSource & 0x08) {
    unsigned long currentTime = millis();
    
    // Software double tap detection - check time since last single tap
    if (currentTime - lastSingleTapTime < DOUBLE_TAP_WINDOW) {
      // This is the second tap within the time window - it's a double tap!
      handleDoubleTap();
    } else {
      // First tap or too long since last tap
      Serial.println("Single tap detected - waiting for second tap");
    }
    
    // Update the last tap time
    lastSingleTapTime = currentTime;
  }
  
  // Additionally detect rapid tap sequences (3+ taps in under 200ms)
  if (tapSequenceActive && tapCount >= 3 && (millis() - tapSequenceStartTime < 200)) {
    Serial.println("*** RAPID TAP SEQUENCE DETECTED! ***");
    Serial.print(tapCount);
    Serial.println(" taps in rapid succession");
    
    // Reset the sequence to avoid multiple triggers
    tapSequenceActive = false;
    
    // Treat rapid tap sequence as double tap
    handleDoubleTap();
  }
  
  // Display detailed information about the tap source register whenever it's non-zero
  if (tapSource != 0) {
    decodeTapSource(tapSource);
  }
  
  // Small delay to prevent flooding the serial monitor
  delay(10);
}

// Apply calibration correction to raw sensor values
float applyCalibration(float value, float bias, float scale) {
  return (value - bias) * scale;
}

// Production-grade IMU calibration routine
void calibrateIMU(bool fullCalibration) {
  const int samples = 500;         // Number of samples for averaging
  const int discardSamples = 100;  // Initial samples to discard (for sensor stabilization)
  const int delayMs = 10;          // Delay between samples (ms)
  
  float accelSum[3] = {0.0f, 0.0f, 0.0f};
  float gyroSum[3] = {0.0f, 0.0f, 0.0f};
  
  // Storage for min/max values for 6-point calibration
  float accelMin[3] = {1000.0f, 1000.0f, 1000.0f};
  float accelMax[3] = {-1000.0f, -1000.0f, -1000.0f};
  
  // LEDs to indicate calibration mode
  setLEDColor(false, false, true); // Blue for calibration
  
  Serial.println("\n=== IMU CALIBRATION STARTED ===");
  Serial.println("Place device on a flat, level surface");
  Serial.println("Keep completely still...");
  
  // Discard initial readings to let sensor stabilize
  for (int i = 0; i < discardSamples; i++) {
    imu.readRawAccelX();
    imu.readRawAccelY();
    imu.readRawAccelZ();
    imu.readRawGyroX();
    imu.readRawGyroY();
    imu.readRawGyroZ();
    delay(delayMs);
  }
  
  // Collect samples for bias calculation (device must remain still)
  Serial.println("Collecting data for bias calibration...");
  Serial.print("[");
  
  for (int i = 0; i < samples; i++) {
    // Show progress every 10%
    if (i % (samples/10) == 0) {
      Serial.print("=");
    }
    
    // Read raw values (direct from sensor)
    float accelX = imu.readFloatAccelX();
    float accelY = imu.readFloatAccelY();
    float accelZ = imu.readFloatAccelZ();
    float gyroX = imu.readFloatGyroX();
    float gyroY = imu.readFloatGyroY();
    float gyroZ = imu.readFloatGyroZ();
    
    // Accumulate values
    accelSum[0] += accelX;
    accelSum[1] += accelY;
    accelSum[2] += accelZ;
    gyroSum[0] += gyroX;
    gyroSum[1] += gyroY;
    gyroSum[2] += gyroZ;
    
    // Update min/max for 6-point calibration
    accelMin[0] = min(accelMin[0], accelX);
    accelMin[1] = min(accelMin[1], accelY);
    accelMin[2] = min(accelMin[2], accelZ);
    accelMax[0] = max(accelMax[0], accelX);
    accelMax[1] = max(accelMax[1], accelY);
    accelMax[2] = max(accelMax[2], accelZ);
    
    delay(delayMs);
  }
  
  Serial.println("]");
  
  // Calculate average (bias) values
  for (int i = 0; i < 3; i++) {
    // For accelerometer: We expect one axis to be ~1g (9.81 m/s²) due to gravity,
    // depending on device orientation. Other axes should be near 0g.
    accelSum[i] /= samples;
    gyroSum[i] /= samples;
  }
  
  // Data collection analysis - check for movement during calibration
  float accelRange[3] = {
    accelMax[0] - accelMin[0],
    accelMax[1] - accelMin[1],
    accelMax[2] - accelMin[2]
  };
  
  bool calibrationValid = true;
  
  // Check if device was too unstable during calibration
  for (int i = 0; i < 3; i++) {
    if (accelRange[i] > 0.1f) { // More than 0.1g variation during calibration
      calibrationValid = false;
      Serial.print("WARNING: Too much movement on accel axis ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(accelRange[i]);
      Serial.println("g variation");
    }
  }
  
  // Calculate accelerometer gravity vector magnitude
  float gravityMag = sqrt(
    accelSum[0] * accelSum[0] + 
    accelSum[1] * accelSum[1] + 
    accelSum[2] * accelSum[2]
  );
  
  Serial.print("Gravity magnitude: ");
  Serial.print(gravityMag, 4);
  Serial.println("g (should be close to 1.0)");
  
  if (abs(gravityMag - 1.0f) > 0.1f) {
    Serial.println("WARNING: Gravity magnitude very different from expected 1g");
    Serial.println("Sensor may need factory calibration or hardware inspection");
  }
  
  // Set gyro bias directly from measurements (gyro should read 0 when still)
  calibrationData.gyroBias[0] = gyroSum[0];
  calibrationData.gyroBias[1] = gyroSum[1];
  calibrationData.gyroBias[2] = gyroSum[2];
  
  // Find which axis has the largest absolute reading (this is likely aligned with gravity)
  int gravityAxis = 0;
  float maxAccel = abs(accelSum[0]);
  for (int i = 1; i < 3; i++) {
    if (abs(accelSum[i]) > maxAccel) {
      maxAccel = abs(accelSum[i]);
      gravityAxis = i;
    }
  }
  
  // Determine sign of gravity on that axis (positive or negative)
  float gravitySign = accelSum[gravityAxis] > 0 ? 1.0f : -1.0f;
  
  // Set accelerometer bias for all axes
  for (int i = 0; i < 3; i++) {
    if (i == gravityAxis) {
      // For gravity-aligned axis, bias is current reading minus expected gravity (1g)
      calibrationData.accelBias[i] = accelSum[i] - (1.0f * gravitySign);
    } else {
      // For other axes, bias is the full measured value (should be 0 when level)
      calibrationData.accelBias[i] = accelSum[i];
    }
  }
  
  // Calculate accelerometer scaling factors
  // In a simple calibration, we set scaling to 1.0
  calibrationData.accelScale[0] = 1.0f;
  calibrationData.accelScale[1] = 1.0f;
  calibrationData.accelScale[2] = 1.0f;
  
  // Scale correction based on gravity vector magnitude
  if (gravityMag > 0.1f) { // Avoid division by zero
    float scaleFactor = 1.0f / gravityMag;
    calibrationData.accelScale[0] = scaleFactor;
    calibrationData.accelScale[1] = scaleFactor;
    calibrationData.accelScale[2] = scaleFactor;
  }
  
  // If full calibration requested, would perform 6-point calibration here
  // (requires rotating device to 6 different orientations)
  if (fullCalibration) {
    // This would be a multi-step process with user interaction
    // Not implemented in this simplified version
    Serial.println("Full 6-point calibration not implemented in this version");
  }
  
  // Mark calibration as complete and record timestamp
  calibrationData.isCalibrated = calibrationValid;
  calibrationData.calibrationTimestamp = millis();
  
  // Print calibration results
  Serial.println("\nCALIBRATION RESULTS:");
  Serial.println("Accelerometer bias (g):");
  Serial.print("  X: "); Serial.println(calibrationData.accelBias[0], 6);
  Serial.print("  Y: "); Serial.println(calibrationData.accelBias[1], 6);
  Serial.print("  Z: "); Serial.println(calibrationData.accelBias[2], 6);
  
  Serial.println("Accelerometer scale factors:");
  Serial.print("  X: "); Serial.println(calibrationData.accelScale[0], 6);
  Serial.print("  Y: "); Serial.println(calibrationData.accelScale[1], 6);
  Serial.print("  Z: "); Serial.println(calibrationData.accelScale[2], 6);
  
  Serial.println("Gyroscope bias (dps):");
  Serial.print("  X: "); Serial.println(calibrationData.gyroBias[0], 6);
  Serial.print("  Y: "); Serial.println(calibrationData.gyroBias[1], 6);
  Serial.print("  Z: "); Serial.println(calibrationData.gyroBias[2], 6);
  
  if (calibrationValid) {
    Serial.println("Calibration SUCCESSFUL");
    // Save to EEPROM in a real application
    // saveCalibrationToEEPROM();
  } else {
    Serial.println("Calibration finished with WARNINGS - results may not be accurate");
    Serial.println("Try again with device on a more stable surface");
  }
  
  Serial.println("=== CALIBRATION COMPLETE ===\n");
  
  // Back to normal LED state
  setLEDColor(true, false, false); // Red - waiting mode
}

// For future implementation - save calibration to EEPROM
void saveCalibrationToEEPROM() {
  // Code to save calibration data to EEPROM goes here
  // (Not implemented in this version)
}

// For future implementation - load calibration from EEPROM
void loadCalibrationFromEEPROM() {
  // Code to load calibration data from EEPROM goes here
  // (Not implemented in this version)
}

// Feature extraction for gesture authentication
void extractFeatures(GestureData* gesture, GestureFeatures* features) {
  float accMagSum = 0;
  float accMagSqSum = 0;
  float gyroMagSum = 0;
  float gyroMagSqSum = 0;
  float accMag[MAX_GESTURE_SAMPLES];
  float gyroMag[MAX_GESTURE_SAMPLES];
  
  // Calculate magnitudes and sums
  for (int i = 0; i < gesture->sampleCount; i++) {
    // Calculate acceleration magnitude
    accMag[i] = calculateAccMagnitude(
      gesture->accX[i], gesture->accY[i], gesture->accZ[i]);
    accMagSum += accMag[i];
    accMagSqSum += accMag[i] * accMag[i];
    
    // Calculate gyroscope magnitude
    gyroMag[i] = calculateGyroMagnitude(
      gesture->gyroX[i], gesture->gyroY[i], gesture->gyroZ[i]);
    gyroMagSum += gyroMag[i];
    gyroMagSqSum += gyroMag[i] * gyroMag[i];
  }
  
  // Calculate basic statistical features
  features->accMagnitudeMean = accMagSum / gesture->sampleCount;
  features->gyroMagnitudeMean = gyroMagSum / gesture->sampleCount;
  
  // Calculate standard deviations
  float accMagVariance = (accMagSqSum / gesture->sampleCount) - 
                         (features->accMagnitudeMean * features->accMagnitudeMean);
  features->accMagnitudeStd = sqrt(accMagVariance > 0 ? accMagVariance : 0);
  
  float gyroMagVariance = (gyroMagSqSum / gesture->sampleCount) - 
                          (features->gyroMagnitudeMean * features->gyroMagnitudeMean);
  features->gyroMagnitudeStd = sqrt(gyroMagVariance > 0 ? gyroMagVariance : 0);
  
  // Simplified approximation of dominant frequency - find most common period
  // This is a very basic approach - a real implementation would use FFT
  int zeroCrossings = 0;
  for (int i = 1; i < gesture->sampleCount; i++) {
    if ((gyroMag[i] > features->gyroMagnitudeMean && 
         gyroMag[i-1] < features->gyroMagnitudeMean) ||
        (gyroMag[i] < features->gyroMagnitudeMean && 
         gyroMag[i-1] > features->gyroMagnitudeMean)) {
      zeroCrossings++;
    }
  }
  
  // Approximate frequency from zero crossings
  if (zeroCrossings > 0) {
    float recordingDurationSec = (float)RECORDING_DURATION / 1000.0f;
    features->dominantFrequency = zeroCrossings / (2 * recordingDurationSec);
  } else {
    features->dominantFrequency = 0;
  }
  
  // Simplified energy ratio - ratio of high to low frequency energy
  // Dividing the signal into first and second half as a simple approach
  float lowFreqEnergy = 0;
  float highFreqEnergy = 0;
  
  int midpoint = gesture->sampleCount / 2;
  for (int i = 1; i < midpoint; i++) {
    float diff = gyroMag[i] - gyroMag[i-1];
    lowFreqEnergy += diff * diff;
  }
  
  for (int i = midpoint + 1; i < gesture->sampleCount; i++) {
    float diff = gyroMag[i] - gyroMag[i-1];
    highFreqEnergy += diff * diff;
  }
  
  if (lowFreqEnergy > 0) {
    features->energyRatio = highFreqEnergy / lowFreqEnergy;
  } else {
    features->energyRatio = 0;
  }
  
  // Print extracted features
  Serial.println("Extracted Features:");
  Serial.print("Acc Magnitude Mean: "); Serial.println(features->accMagnitudeMean, 4);
  Serial.print("Acc Magnitude Std: "); Serial.println(features->accMagnitudeStd, 4);
  Serial.print("Gyro Magnitude Mean: "); Serial.println(features->gyroMagnitudeMean, 4);
  Serial.print("Gyro Magnitude Std: "); Serial.println(features->gyroMagnitudeStd, 4);
  Serial.print("Dominant Frequency: "); Serial.println(features->dominantFrequency, 4);
  Serial.print("Energy Ratio: "); Serial.println(features->energyRatio, 4);
}

// Calculate authentication score between reference and current gesture
float calculateAuthenticationScore(GestureFeatures* ref, GestureFeatures* current) {
  // Simple feature similarity calculation
  float scores[NUM_AUTH_FEATURES];
  
  // Normalized absolute difference for each feature
  scores[0] = 1.0f - min(1.0f, abs(ref->accMagnitudeMean - current->accMagnitudeMean) / max(ref->accMagnitudeMean, 0.1f));
  scores[1] = 1.0f - min(1.0f, abs(ref->accMagnitudeStd - current->accMagnitudeStd) / max(ref->accMagnitudeStd, 0.1f));
  scores[2] = 1.0f - min(1.0f, abs(ref->gyroMagnitudeMean - current->gyroMagnitudeMean) / max(ref->gyroMagnitudeMean, 0.1f));
  scores[3] = 1.0f - min(1.0f, abs(ref->gyroMagnitudeStd - current->gyroMagnitudeStd) / max(ref->gyroMagnitudeStd, 0.1f));
  scores[4] = 1.0f - min(1.0f, abs(ref->dominantFrequency - current->dominantFrequency) / max(ref->dominantFrequency, 0.1f));
  scores[5] = 1.0f - min(1.0f, abs(ref->energyRatio - current->energyRatio) / max(ref->energyRatio, 0.1f));
  
  // Weighted average
  float weights[NUM_AUTH_FEATURES] = {0.15f, 0.2f, 0.15f, 0.2f, 0.15f, 0.15f};
  float totalScore = 0;
  
  for (int i = 0; i < NUM_AUTH_FEATURES; i++) {
    totalScore += scores[i] * weights[i];
    
    // Print individual feature scores
    Serial.print("Feature ");
    Serial.print(i);
    Serial.print(" score: ");
    Serial.println(scores[i], 4);
  }
  
  return totalScore;
}

// Toggle between enrollment and authentication modes
void setAuthenticationMode(bool isAuth) {
  authenticationMode = isAuth;
  
  if (isAuth) {
    Serial.println("*** AUTHENTICATION MODE ACTIVATED ***");
    Serial.println("Use your recorded gesture to authenticate");
    
    // Check if we have reference gestures
    bool hasReference = false;
    for (int i = 0; i < 3; i++) {
      if (referenceGestures[i].isActive) {
        hasReference = true;
        break;
      }
    }
    
    if (!hasReference) {
      Serial.println("ERROR: No reference gestures found.");
      Serial.println("Please enroll a gesture first (use 'enroll' command)");
      authenticationMode = false;
      setLEDColor(true, false, false); // Red for waiting mode
      return;
    }
    
    setLEDColor(false, false, true); // Blue for authentication mode
  } else {
    Serial.println("*** ENROLLMENT MODE ACTIVATED ***");
    Serial.println("Record 3 samples of your gesture to enroll");
    setLEDColor(true, false, false); // Red for enrollment/waiting
  }
}