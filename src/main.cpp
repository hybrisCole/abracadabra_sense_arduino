#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>

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
#define RECORDING_DURATION 4000  // Recording window duration in ms (4 seconds)

// Sampling configuration
#define SAMPLE_RATE_MS 20  // Sample every 20ms (approximately 50Hz)
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;

// Low-pass filter variables
float prevAccX = 0, prevAccY = 0, prevAccZ = 0;
float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;
#define FILTER_ALPHA 0.05  // Filter coefficient (0-1), lower = more filtering (changed from 0.2)

// Gyroscope calibration variables
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
bool gyroCalibrated = false;

// Gesture processing constants
#define MOVEMENT_THRESHOLD 0.1      // Threshold to detect start of gesture (g)
#define NOISE_FLOOR 0.03            // Noise floor for IMU when still (g)
#define MAX_GESTURE_SAMPLES 200     // Maximum samples in a processed gesture (4sec @ 50Hz)

// Gesture reference storage
typedef struct {
  float accX[MAX_GESTURE_SAMPLES];
  float accY[MAX_GESTURE_SAMPLES];
  float accZ[MAX_GESTURE_SAMPLES];
  float gyroX[MAX_GESTURE_SAMPLES];
  float gyroY[MAX_GESTURE_SAMPLES];
  float gyroZ[MAX_GESTURE_SAMPLES];
  int sampleCount;
  bool isActive;
} GestureData;

GestureData referenceGestures[3];  // 3 reference gestures
GestureData currentGesture;        // Current gesture being recorded
int currentReferenceIndex = 0;     // Current reference gesture index (0-2)
bool inComparisonMode = false;     // True when comparing, false when recording references

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
  uint8_t tapCfg, tapThs, intDur2, wakeUpThs, ctrl10, ctrl1;
  
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

// Calibrate gyroscope to reduce drift
void calibrateGyroscope() {
  Serial.println("\nCalibrating gyroscope...");
  Serial.println("Keep the device still for 2 seconds");
  
  // Visual indicator that calibration is starting
  setLEDColor(false, false, true);  // Blue LED during calibration
  
  // Wait a moment for the device to settle
  delay(500);
  
  // Variables for averaging
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  const int numSamples = 100;
  
  // Collect samples
  for (int i = 0; i < numSamples; i++) {
    sumGyroX += imu.readFloatGyroX();
    sumGyroY += imu.readFloatGyroY();
    sumGyroZ += imu.readFloatGyroZ();
    delay(20);  // Sample at 50Hz
  }
  
  // Calculate average offsets
  gyroXoffset = sumGyroX / numSamples;
  gyroYoffset = sumGyroY / numSamples;
  gyroZoffset = sumGyroZ / numSamples;
  
  // Mark as calibrated
  gyroCalibrated = true;
  
  // Print calibration results
  Serial.println("Gyroscope calibration complete");
  Serial.print("X-axis offset: "); Serial.println(gyroXoffset, 4);
  Serial.print("Y-axis offset: "); Serial.println(gyroYoffset, 4);
  Serial.print("Z-axis offset: "); Serial.println(gyroZoffset, 4);
  
  // Visual indicator that calibration is complete
  setLEDColor(true, false, false);  // Back to red (waiting state)
}

// Apply low-pass filter to sensor data
float applyLowPassFilter(float currentValue, float prevValue) {
  return FILTER_ALPHA * currentValue + (1.0 - FILTER_ALPHA) * prevValue;
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
  Serial.print("- Low-pass filter coefficient: "); Serial.println(FILTER_ALPHA);
  
  // Add gyroscope calibration information
  Serial.println("\nGyroscope Calibration:");
  Serial.print("- X-axis offset: "); Serial.println(gyroXoffset, 4);
  Serial.print("- Y-axis offset: "); Serial.println(gyroYoffset, 4);
  Serial.print("- Z-axis offset: "); Serial.println(gyroZoffset, 4);
  
  Serial.println("\nSampling Considerations:");
  Serial.println("* Sample rate: 50-100Hz is typically sufficient for human gestures");
  Serial.println("* Duration: Balance between memory constraints and gesture complexity");
  Serial.println("* Filtering: Simple low-pass filter applied to reduce noise");
  
  Serial.println("------------------------------");
}

// Handle double tap detection - now manages recording window
void handleDoubleTap() {
  Serial.println("*** DOUBLE TAP DETECTED! ***");

  // Set flag to ignore taps during gesture recording
  ignoreTaps = true;
  
  // Different behavior based on current state
  if (!inComparisonMode) {
    // We're in reference recording mode
    Serial.print("Recording REFERENCE gesture #");
    Serial.println(currentReferenceIndex + 1);
    
    // Start recording window
    isRecording = true;
    recordingStartTime = millis();
    sampleCount = 0;
    
    // Reset current gesture data
    currentGesture.isActive = true;
    currentGesture.sampleCount = 0;
    
    // Print metadata and CSV header for the data
    printRecordingMetadata();
    Serial.println("\nRECORDING STARTED - 4 second window");
    Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
    
    // Set LED to green during recording
    setLEDColor(false, true, false);
    
    // Reset filter values
    prevAccX = 0; prevAccY = 0; prevAccZ = 0;
    prevGyroX = 0; prevGyroY = 0; prevGyroZ = 0;
  } 
  else {
    // We're in comparison mode
    Serial.println("Comparing gesture to references...");
    
    // Start recording window for comparison
    isRecording = true;
    recordingStartTime = millis();
    sampleCount = 0;
    
    // Reset current gesture data
    currentGesture.isActive = true;
    currentGesture.sampleCount = 0;
    
    // Print metadata and CSV header for the data
    printRecordingMetadata();
    Serial.println("\nCOMPARING GESTURE - 4 second window");
    Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
    
    // Set LED to purple during comparison recording
    setLEDColor(true, false, true);
    
    // Reset filter values
    prevAccX = 0; prevAccY = 0; prevAccZ = 0;
    prevGyroX = 0; prevGyroY = 0; prevGyroZ = 0;
  }
}

// Read and print sensor data
void recordSensorData() {
  // Only record if we have space
  if (sampleCount >= MAX_GESTURE_SAMPLES) return;
  
  // Get current time
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - recordingStartTime;
  
  // Get IMU data
  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  
  // Get gyroscope data and apply calibration offset
  float gyroX = imu.readFloatGyroX() - gyroXoffset;
  float gyroY = imu.readFloatGyroY() - gyroYoffset;
  float gyroZ = imu.readFloatGyroZ() - gyroZoffset;
  
  // Apply low-pass filter
  accX = applyLowPassFilter(accX, prevAccX);
  accY = applyLowPassFilter(accY, prevAccY);
  accZ = applyLowPassFilter(accZ, prevAccZ);
  
  gyroX = applyLowPassFilter(gyroX, prevGyroX);
  gyroY = applyLowPassFilter(gyroY, prevGyroY);
  gyroZ = applyLowPassFilter(gyroZ, prevGyroZ);
  
  // Store current values for next filter iteration
  prevAccX = accX; prevAccY = accY; prevAccZ = accZ;
  prevGyroX = gyroX; prevGyroY = gyroY; prevGyroZ = gyroZ;
  
  // Save to currentGesture structure
  currentGesture.accX[sampleCount] = accX;
  currentGesture.accY[sampleCount] = accY;
  currentGesture.accZ[sampleCount] = accZ;
  currentGesture.gyroX[sampleCount] = gyroX;
  currentGesture.gyroY[sampleCount] = gyroY;
  currentGesture.gyroZ[sampleCount] = gyroZ;
  
  // Print in CSV format: timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
  Serial.print(elapsedTime);
  Serial.print(",");
  Serial.print(accX, 4);  // 4 decimal places
  Serial.print(",");
  Serial.print(accY, 4);
  Serial.print(",");
  Serial.print(accZ, 4);
  Serial.print(",");
  Serial.print(gyroX, 4);
  Serial.print(",");
  Serial.print(gyroY, 4);
  Serial.print(",");
  Serial.println(gyroZ, 4);
  
  // Increment sample count
  sampleCount++;
  currentGesture.sampleCount = sampleCount;
  
  // Update last sample time
  lastSampleTime = currentTime;
}

// Detect the actual start of movement in a gesture
int detectGestureStart(GestureData* gesture) {
  // Calculate the baseline noise from the first few samples
  float baselineAcc = 0;
  for (int i = 0; i < 5 && i < gesture->sampleCount; i++) {
    baselineAcc += sqrt(sq(gesture->accX[i]) + sq(gesture->accY[i]) + sq(gesture->accZ[i] - 1.0));
  }
  baselineAcc /= 5;
  
  // Look for the first significant movement above the noise floor
  for (int i = 5; i < gesture->sampleCount; i++) {
    // Calculate magnitude of acceleration change (removing gravity from Z)
    float accMagnitude = sqrt(sq(gesture->accX[i]) + sq(gesture->accY[i]) + sq(gesture->accZ[i] - 1.0));
    
    // If we detect movement above threshold
    if (accMagnitude > baselineAcc + MOVEMENT_THRESHOLD) {
      // Return a few samples before the detected start
      return max(0, i - 3);
    }
  }
  
  // If no clear start found, return 0
  return 0;
}

// Normalize gesture data to range -1 to 1
void normalizeGesture(GestureData* gesture) {
  // Find min/max values for each axis
  float minAcc[3] = {1000, 1000, 1000};
  float maxAcc[3] = {-1000, -1000, -1000};
  float minGyro[3] = {1000, 1000, 1000};
  float maxGyro[3] = {-1000, -1000, -1000};
  
  // Find min/max for each axis
  for (int i = 0; i < gesture->sampleCount; i++) {
    // Accelerometer
    minAcc[0] = min(minAcc[0], gesture->accX[i]);
    maxAcc[0] = max(maxAcc[0], gesture->accX[i]);
    
    minAcc[1] = min(minAcc[1], gesture->accY[i]);
    maxAcc[1] = max(maxAcc[1], gesture->accY[i]);
    
    minAcc[2] = min(minAcc[2], gesture->accZ[i]);
    maxAcc[2] = max(maxAcc[2], gesture->accZ[i]);
    
    // Gyroscope
    minGyro[0] = min(minGyro[0], gesture->gyroX[i]);
    maxGyro[0] = max(maxGyro[0], gesture->gyroX[i]);
    
    minGyro[1] = min(minGyro[1], gesture->gyroY[i]);
    maxGyro[1] = max(maxGyro[1], gesture->gyroY[i]);
    
    minGyro[2] = min(minGyro[2], gesture->gyroZ[i]);
    maxGyro[2] = max(maxGyro[2], gesture->gyroZ[i]);
  }
  
  // Normalize each value to range -1 to 1
  for (int i = 0; i < gesture->sampleCount; i++) {
    // Accelerometer normalization
    if (maxAcc[0] != minAcc[0]) gesture->accX[i] = 2.0 * (gesture->accX[i] - minAcc[0]) / (maxAcc[0] - minAcc[0]) - 1.0;
    if (maxAcc[1] != minAcc[1]) gesture->accY[i] = 2.0 * (gesture->accY[i] - minAcc[1]) / (maxAcc[1] - minAcc[1]) - 1.0;
    if (maxAcc[2] != minAcc[2]) gesture->accZ[i] = 2.0 * (gesture->accZ[i] - minAcc[2]) / (maxAcc[2] - minAcc[2]) - 1.0;
    
    // Gyroscope normalization
    if (maxGyro[0] != minGyro[0]) gesture->gyroX[i] = 2.0 * (gesture->gyroX[i] - minGyro[0]) / (maxGyro[0] - minGyro[0]) - 1.0;
    if (maxGyro[1] != minGyro[1]) gesture->gyroY[i] = 2.0 * (gesture->gyroY[i] - minGyro[1]) / (maxGyro[1] - minGyro[1]) - 1.0;
    if (maxGyro[2] != minGyro[2]) gesture->gyroZ[i] = 2.0 * (gesture->gyroZ[i] - minGyro[2]) / (maxGyro[2] - minGyro[2]) - 1.0;
  }
}

// Process the raw gesture data for comparison
void processGestureData(GestureData* gesture) {
  // Only process if we have data
  if (gesture->sampleCount == 0 || !gesture->isActive) return;
  
  // Step 1: Find the actual start of the gesture
  int startIdx = detectGestureStart(gesture);
  
  // Step 2: If we need to shift data to the beginning
  if (startIdx > 0) {
    // Shift data to the beginning of the array
    for (int i = 0; i < gesture->sampleCount - startIdx; i++) {
      gesture->accX[i] = gesture->accX[i + startIdx];
      gesture->accY[i] = gesture->accY[i + startIdx];
      gesture->accZ[i] = gesture->accZ[i + startIdx];
      gesture->gyroX[i] = gesture->gyroX[i + startIdx];
      gesture->gyroY[i] = gesture->gyroY[i + startIdx];
      gesture->gyroZ[i] = gesture->gyroZ[i + startIdx];
    }
    // Update sample count
    gesture->sampleCount -= startIdx;
  }
  
  // Step 3: Truncate to a maximum size if needed
  if (gesture->sampleCount > MAX_GESTURE_SAMPLES) {
    gesture->sampleCount = MAX_GESTURE_SAMPLES;
  }
  
  // Step 4: Normalize the data to a standard range
  normalizeGesture(gesture);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("XIAO nRF52840 Sense - Gesture Recognition");
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
  configureTapDetection();
  
  // Calibrate gyroscope
  calibrateGyroscope();
  
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
  // Check if we should record sensor data during recording window
  if (isRecording) {
    unsigned long currentTime = millis();
    
    // Sample at our defined rate and print the data
    if (currentTime - lastSampleTime >= SAMPLE_RATE_MS) {
      recordSensorData();
    }
    
    // Check if recording window has ended
    if (currentTime - recordingStartTime >= RECORDING_DURATION) {
      // Recording window ended
      isRecording = false;
      Serial.print("\nRECORDING FINISHED - ");
      Serial.print(sampleCount);
      Serial.println(" samples collected");
      
      // Process the recorded gesture data
      processGestureData(&currentGesture);
      
      // Store reference or compare based on mode
      if (!inComparisonMode) {
        // Store this as a reference gesture
        memcpy(&referenceGestures[currentReferenceIndex], &currentGesture, sizeof(GestureData));
        referenceGestures[currentReferenceIndex].isActive = true;
        
        // Move to next reference or switch to comparison mode
        currentReferenceIndex++;
        if (currentReferenceIndex >= 3) {
          // We've recorded all references, switch to comparison mode
          inComparisonMode = true;
          currentReferenceIndex = 0;
          Serial.println("\n*** All reference gestures recorded! ***");
          Serial.println("Switching to COMPARISON mode. Double-tap to compare gestures.");
          
          // Blue LED indicates comparison mode
          setLEDColor(false, false, true);
        } else {
          // More references to record
          Serial.print("\nReady for next reference gesture. Double-tap to record reference #");
          Serial.println(currentReferenceIndex + 1);
          
          // Return to waiting state (red LED)
          setLEDColor(true, false, false);
        }
      } 
      else {
        // We're in comparison mode, compare with references
        // This will be implemented in the next step
        // For now, just acknowledge
        Serial.println("\nGesture recorded for comparison");
        Serial.println("Comparison algorithm will be implemented next");
        
        // Return to comparison mode waiting state (blue LED)
        setLEDColor(false, false, true);
      }
      
      // Re-enable tap detection
      ignoreTaps = false;
    }
    
    // During recording, we only focus on collecting data, not detecting taps
    return;
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