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
#define COUNTDOWN_TIME 3       // Countdown in seconds before recording starts
#define MOTION_THRESHOLD 0.3   // Maximum allowed motion during pre-recording check (increased from 0.15)
#define SKIP_STABILITY_CHECK true  // Set to true to skip stability check if device is too sensitive

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
#define SAMPLE_STEP 2               // Process every Nth sample to reduce computation
#define SIMILARITY_THRESHOLD 70.0   // Similarity percentage threshold for a match

// TinyML data collection enhancements
#define MAX_GESTURE_TYPES 5         // Maximum number of gesture types (for model training)
#define GESTURE_NAME_LENGTH 20      // Maximum length of gesture name string
char gestureNames[MAX_GESTURE_TYPES][GESTURE_NAME_LENGTH] = {
  "circle",            // ID: 0
  "swipe_left_right",  // ID: 1
  "swipe_up_down",     // ID: 2
  "figure_eight",      // ID: 3
  "zigzag"             // ID: 4
};
int currentGestureID = 0;           // Current gesture type ID (0-4)
int repetitionCount = 0;            // Counter for repetitions of each gesture type
#define REQUIRED_REPETITIONS 5      // How many examples of each gesture to collect

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
  int gestureID;                    // Gesture type ID (0-4)
  int repetition;                   // Which repetition this is (1-5)
} GestureData;

GestureData referenceGestures[3];  // 3 reference gestures
GestureData currentGesture;        // Current gesture being recorded
int currentReferenceIndex = 0;     // Current reference gesture index (0-2)
bool inComparisonMode = false;     // True when comparing, false when recording references
bool inTrainingMode = false;       // True when collecting data for TinyML training

// Constants for weighting different sensor components
#define ACCEL_WEIGHT 0.7    // Weight for accelerometer (70%)
#define GYRO_WEIGHT 0.3     // Weight for gyroscope (30%)

// Gesture results
int bestMatchIndex = -1;    // Index of best matching reference gesture
float bestMatchScore = 0;   // Similarity score for the best match (0-100)

// IMU noise floor calibration
float noiseFloorAccX = 0, noiseFloorAccY = 0, noiseFloorAccZ = 0;
bool imuNoiseCalibrated = false;

// Forward declarations
float compareGestures(GestureData* gesture1, GestureData* gesture2);
float compareAxis(float* series1, int len1, float* series2, int len2);
float calculateSimilarity(float distance);
bool checkDeviceStability();
void performCountdown();
void showRecordingProgress(unsigned long elapsedTime);
void updateProgressLED(unsigned long elapsedTime);
void calibrateAccelerometerNoise();

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
    int index2 = min(index1 + 1, gesture->sampleCount - 1);
    float fraction = srcIndex - index1;
    
    // Linear interpolation between samples
    outputAcc[j] = accMag[index1] * (1 - fraction) + accMag[index2] * fraction;
    outputGyro[j] = gyroMag[index1] * (1 - fraction) + gyroMag[index2] * fraction;
  }
}

// Compare two gestures using a simple template matching approach
float compareGestures(GestureData* gesture1, GestureData* gesture2) {
  if (!gesture1->isActive || !gesture2->isActive || 
      gesture1->sampleCount < 10 || gesture2->sampleCount < 10) {
    return 0.0; // Invalid data
  }
  
  // Use a fixed length for comparison (50 samples)
  const int fixedLength = 50;
  float accMag1[fixedLength], gyroMag1[fixedLength];
  float accMag2[fixedLength], gyroMag2[fixedLength];
  
  // Resample both gestures to same length
  resampleGesture(gesture1, accMag1, gyroMag1, fixedLength);
  resampleGesture(gesture2, accMag2, gyroMag2, fixedLength);
  
  // Calculate normalized cross-correlation for acceleration and gyroscope
  float accCorrelation = 0;
  float gyroCorrelation = 0;
  
  // Calculate mean
  float accMean1 = 0, accMean2 = 0;
  float gyroMean1 = 0, gyroMean2 = 0;
  
  for (int i = 0; i < fixedLength; i++) {
    accMean1 += accMag1[i];
    accMean2 += accMag2[i];
    gyroMean1 += gyroMag1[i];
    gyroMean2 += gyroMag2[i];
  }
  
  accMean1 /= fixedLength;
  accMean2 /= fixedLength;
  gyroMean1 /= fixedLength;
  gyroMean2 /= fixedLength;
  
  // Calculate numerator and denominator for correlation
  float accNum = 0, accDenom1 = 0, accDenom2 = 0;
  float gyroNum = 0, gyroDenom1 = 0, gyroDenom2 = 0;
  
  for (int i = 0; i < fixedLength; i++) {
    // Acceleration correlation
    float accDiff1 = accMag1[i] - accMean1;
    float accDiff2 = accMag2[i] - accMean2;
    accNum += accDiff1 * accDiff2;
    accDenom1 += accDiff1 * accDiff1;
    accDenom2 += accDiff2 * accDiff2;
    
    // Gyroscope correlation
    float gyroDiff1 = gyroMag1[i] - gyroMean1;
    float gyroDiff2 = gyroMag2[i] - gyroMean2;
    gyroNum += gyroDiff1 * gyroDiff2;
    gyroDenom1 += gyroDiff1 * gyroDiff1;
    gyroDenom2 += gyroDiff2 * gyroDiff2;
  }
  
  // Calculate correlation coefficient (-1 to 1)
  if (accDenom1 > 0 && accDenom2 > 0) {
    accCorrelation = accNum / (sqrt(accDenom1) * sqrt(accDenom2));
  }
  
  if (gyroDenom1 > 0 && gyroDenom2 > 0) {
    gyroCorrelation = gyroNum / (sqrt(gyroDenom1) * sqrt(gyroDenom2));
  }
  
  // Convert correlation to similarity score (0-100%)
  // Correlation ranges from -1 to 1, where 1 is perfect match
  float accSimilarity = 50.0 * (accCorrelation + 1.0);
  float gyroSimilarity = 50.0 * (gyroCorrelation + 1.0);
  
  // Apply weighting between accelerometer and gyroscope
  float similarity = (accSimilarity * ACCEL_WEIGHT) + (gyroSimilarity * GYRO_WEIGHT);
  
  return constrain(similarity, 0.0, 100.0);
}

// Compare current gesture to all references and find best match
int findBestMatch(GestureData* currentGesture) {
  float bestSimilarity = 0;
  int bestIndex = -1;
  
  Serial.println("\n--- Gesture Comparison Results ---");
  
  // Compare with each reference gesture
  for (int i = 0; i < 3; i++) {
    if (!referenceGestures[i].isActive) continue;
    
    // Calculate similarity
    float similarity = compareGestures(&referenceGestures[i], currentGesture);
    
    // Print match details
    Serial.print("Reference #");
    Serial.print(i+1);
    Serial.print(": Similarity = ");
    Serial.print(similarity, 1);
    Serial.println("%");
    
    // Update best match if this is better
    if (similarity > bestSimilarity) {
      bestSimilarity = similarity;
      bestIndex = i;
      bestMatchScore = similarity;
    }
  }
  
  return bestIndex;
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
  
  // Output gesture type information if in training mode
  if (inTrainingMode) {
    Serial.print("Gesture type: ");
    Serial.print(currentGestureID);
    Serial.print(" (");
    Serial.print(gestureNames[currentGestureID]);
    Serial.println(")");
    
    Serial.print("Repetition: ");
    Serial.print(repetitionCount);
    Serial.print(" of ");
    Serial.println(REQUIRED_REPETITIONS);
  }
  
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
  
  // Flash LED to indicate detected double tap
  for (int i = 0; i < 2; i++) {
    ledOff();
    delay(75);
    setLEDColor(true, true, true); // White flash
    delay(75);
  }
  ledOff();
  
  // Add delay before starting recording to avoid accidental motion
  Serial.println("Preparing to record in 350ms...");
  delay(350);  // 350ms pause before starting recording
  
  // Check if device is stable enough to start recording
  if (!checkDeviceStability()) {
    Serial.println("Too much motion detected! Please hold the device still.");
    // Flash red LED to indicate error
    for (int i = 0; i < 3; i++) {
      setLEDColor(true, false, false); // Red
      delay(100);
      ledOff();
      delay(100);
    }
    ignoreTaps = false;
    return;
  }
  
  // Different behavior based on current state
  if (inTrainingMode) {
    // We're in training data collection mode
    Serial.print("Recording TRAINING data for gesture: ");
    Serial.print(gestureNames[currentGestureID]);
    Serial.print(" (ID: ");
    Serial.print(currentGestureID);
    Serial.print(", repetition ");
    Serial.print(repetitionCount + 1);
    Serial.println(")");
    
    // Countdown before recording with filter warm-up
    performCountdown();
    
    // Start recording window
    isRecording = true;
    recordingStartTime = millis();
    sampleCount = 0;
    
    // Reset current gesture data
    currentGesture.isActive = true;
    currentGesture.sampleCount = 0;
    currentGesture.gestureID = currentGestureID;
    currentGesture.repetition = repetitionCount + 1;
    
    // Print metadata and CSV header for the data
    printRecordingMetadata();
    Serial.println("\nRECORDING STARTED - 4 second window");
    Serial.println("gesture_id,repetition,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
    
    // Set LED to green during recording
    setLEDColor(false, true, false);
  }
  else if (!inComparisonMode) {
    // We're in reference recording mode
    Serial.print("Recording REFERENCE gesture #");
    Serial.println(currentReferenceIndex + 1);
    
    // Countdown before recording with filter warm-up
    performCountdown();
    
    // Start recording window
    isRecording = true;
    recordingStartTime = millis();
    sampleCount = 0;
    
    // Reset current gesture data
    currentGesture.isActive = true;
    currentGesture.sampleCount = 0;
    currentGesture.gestureID = currentReferenceIndex;
    currentGesture.repetition = 1;
    
    // Print metadata and CSV header for the data
    printRecordingMetadata();
    Serial.println("\nRECORDING STARTED - 4 second window");
    Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
    
    // Set LED to green during recording
    setLEDColor(false, true, false);
  } 
  else {
    // We're in comparison mode
    Serial.println("Comparing gesture to references...");
    
    // Countdown before recording with filter warm-up
    performCountdown();
    
    // Start recording window for comparison
    isRecording = true;
    recordingStartTime = millis();
    sampleCount = 0;
    
    // Reset current gesture data
    currentGesture.isActive = true;
    currentGesture.sampleCount = 0;
    currentGesture.gestureID = -1; // Unknown for comparison
    currentGesture.repetition = 0;
    
    // Print metadata and CSV header for the data
    printRecordingMetadata();
    Serial.println("\nCOMPARING GESTURE - 4 second window");
    Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
    
    // Set LED to purple during comparison recording
    setLEDColor(true, false, true);
  }
}

// Check if device is stable enough to start recording
bool checkDeviceStability() {
  // Skip check if configured to do so
  if (SKIP_STABILITY_CHECK) {
    Serial.println("Stability check skipped by configuration");
    return true;
  }
  
  Serial.println("Checking device stability...");
  
  // Set LED to cyan during stability check
  setLEDColor(false, true, true);
  
  // Check for excessive motion over 500ms
  const int checkDuration = 500; // ms
  const int numSamples = 10;
  unsigned long checkStart = millis();
  float totalMotion = 0;
  int sampleCount = 0;
  float maxMotion = 0;
  
  // Sample the accelerometer a few times
  for (int i = 0; i < numSamples && (millis() - checkStart < checkDuration); i++) {
    float accX = imu.readFloatAccelX();
    float accY = imu.readFloatAccelY();
    float accZ = imu.readFloatAccelZ();
    
    // Apply calibration offsets if available
    if (imuNoiseCalibrated) {
      accX -= noiseFloorAccX;
      accY -= noiseFloorAccY;
      accZ -= noiseFloorAccZ;
    }
    
    // Remove gravity component from Z
    float zWithoutGravity = accZ - 1.0;
    
    // Calculate total acceleration magnitude
    float magnitude = sqrt(sq(accX) + sq(accY) + sq(zWithoutGravity));
    
    // Update maximum motion
    if (magnitude > maxMotion) {
      maxMotion = magnitude;
    }
    
    // Add to total for average
    totalMotion += magnitude;
    sampleCount++;
    
    delay(checkDuration / numSamples);
  }
  
  // Calculate average motion
  float avgMotion = totalMotion / sampleCount;
  
  Serial.print("Average motion: ");
  Serial.print(avgMotion, 4);
  Serial.print(", Maximum motion: ");
  Serial.println(maxMotion, 4);
  
  // Consider the average motion rather than just maximum
  // This makes the check more tolerant of occasional spikes
  // but still fails if there's consistent motion
  return avgMotion < (MOTION_THRESHOLD * 0.7) && maxMotion < MOTION_THRESHOLD;
}

// Perform a visual countdown with LED and serial output with sensor warm-up
void performCountdown() {
  Serial.println("Countdown to recording:");
  
  // Warm up the sensor filters before recording
  Serial.println("Warming up sensors...");
  
  // Take some initial readings to prime the filter
  // This eliminates the ramp-up effect on the Z-axis
  const int warmupSamples = 10;
  
  // Read first sample to initialize filters
  prevAccX = imu.readFloatAccelX();
  prevAccY = imu.readFloatAccelY();
  prevAccZ = imu.readFloatAccelZ();
  
  prevGyroX = imu.readFloatGyroX() - gyroXoffset;
  prevGyroY = imu.readFloatGyroY() - gyroYoffset;
  prevGyroZ = imu.readFloatGyroZ() - gyroZoffset;
  
  // Apply calibration offsets to accelerometer if available
  if (imuNoiseCalibrated) {
    prevAccX -= noiseFloorAccX;
    prevAccY -= noiseFloorAccY;
    prevAccZ -= noiseFloorAccZ;
  }
  
  Serial.print("Initial readings - Acc: (");
  Serial.print(prevAccX, 4); Serial.print(", ");
  Serial.print(prevAccY, 4); Serial.print(", ");
  Serial.print(prevAccZ, 4); Serial.print(") Gyro: (");
  Serial.print(prevGyroX, 4); Serial.print(", ");
  Serial.print(prevGyroY, 4); Serial.print(", ");
  Serial.print(prevGyroZ, 4); Serial.println(")");
  
  // Continue filter warm-up with several more samples
  for (int i = 0; i < warmupSamples; i++) {
    // Read sensor data
    float accX = imu.readFloatAccelX();
    float accY = imu.readFloatAccelY();
    float accZ = imu.readFloatAccelZ();
    
    float gyroX = imu.readFloatGyroX() - gyroXoffset;
    float gyroY = imu.readFloatGyroY() - gyroYoffset;
    float gyroZ = imu.readFloatGyroZ() - gyroZoffset;
    
    // Apply calibration offsets if available
    if (imuNoiseCalibrated) {
      accX -= noiseFloorAccX;
      accY -= noiseFloorAccY;
      accZ -= noiseFloorAccZ;
    }
    
    // Apply low-pass filter to stabilize values
    prevAccX = FILTER_ALPHA * accX + (1.0 - FILTER_ALPHA) * prevAccX;
    prevAccY = FILTER_ALPHA * accY + (1.0 - FILTER_ALPHA) * prevAccY;
    prevAccZ = FILTER_ALPHA * accZ + (1.0 - FILTER_ALPHA) * prevAccZ;
    
    prevGyroX = FILTER_ALPHA * gyroX + (1.0 - FILTER_ALPHA) * prevGyroX;
    prevGyroY = FILTER_ALPHA * gyroY + (1.0 - FILTER_ALPHA) * prevGyroY;
    prevGyroZ = FILTER_ALPHA * gyroZ + (1.0 - FILTER_ALPHA) * prevGyroZ;
    
    delay(20); // Brief delay between samples
  }
  
  Serial.print("Warmed up - Acc: (");
  Serial.print(prevAccX, 4); Serial.print(", ");
  Serial.print(prevAccY, 4); Serial.print(", ");
  Serial.print(prevAccZ, 4); Serial.print(") Gyro: (");
  Serial.print(prevGyroX, 4); Serial.print(", ");
  Serial.print(prevGyroY, 4); Serial.print(", ");
  Serial.print(prevGyroZ, 4); Serial.println(")");
  
  // Now perform the countdown
  for (int i = COUNTDOWN_TIME; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    
    // Flash LED for each second of countdown
    setLEDColor(true, true, false); // Yellow
    delay(200);
    ledOff();
    delay(800);
  }
  
  // Signal start
  Serial.println("BEGIN!");
  setLEDColor(false, true, false); // Green - recording starts
}

// Record and print current sensor data
void recordSensorData() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - recordingStartTime;
  
  // Read raw sensor data
  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  float gyroX = imu.readFloatGyroX();
  float gyroY = imu.readFloatGyroY();
  float gyroZ = imu.readFloatGyroZ();
  
  // Apply calibration offsets to gyroscope data
  if (gyroCalibrated) {
    gyroX -= gyroXoffset;
    gyroY -= gyroYoffset;
    gyroZ -= gyroZoffset;
  }
  
  // Apply calibration offsets to accelerometer if available
  if (imuNoiseCalibrated) {
    accX -= noiseFloorAccX;
    accY -= noiseFloorAccY;
    accZ -= noiseFloorAccZ;
  }
  
  // Apply low-pass filter
  accX = FILTER_ALPHA * accX + (1.0 - FILTER_ALPHA) * prevAccX;
  accY = FILTER_ALPHA * accY + (1.0 - FILTER_ALPHA) * prevAccY;
  accZ = FILTER_ALPHA * accZ + (1.0 - FILTER_ALPHA) * prevAccZ;
  gyroX = FILTER_ALPHA * gyroX + (1.0 - FILTER_ALPHA) * prevGyroX;
  gyroY = FILTER_ALPHA * gyroY + (1.0 - FILTER_ALPHA) * prevGyroY;
  gyroZ = FILTER_ALPHA * gyroZ + (1.0 - FILTER_ALPHA) * prevGyroZ;
  
  // Update previous values for next filtering
  prevAccX = accX;
  prevAccY = accY;
  prevAccZ = accZ;
  prevGyroX = gyroX;
  prevGyroY = gyroY;
  prevGyroZ = gyroZ;
  
  // Store in current gesture structure
  currentGesture.accX[sampleCount] = accX;
  currentGesture.accY[sampleCount] = accY;
  currentGesture.accZ[sampleCount] = accZ;
  currentGesture.gyroX[sampleCount] = gyroX;
  currentGesture.gyroY[sampleCount] = gyroY;
  currentGesture.gyroZ[sampleCount] = gyroZ;
  
  // Print in CSV format
  if (inTrainingMode) {
    // Format: gesture_id,repetition,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
    Serial.print(currentGesture.gestureID);
    Serial.print(",");
    Serial.print(currentGesture.repetition);
    Serial.print(",");
  }
  
  // Common CSV format with or without the training prefix
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

// Add accelerometer noise floor calibration function
void calibrateAccelerometerNoise() {
  Serial.println("\nCalibrating accelerometer noise floor...");
  Serial.println("Keep the device completely still");
  
  // Visual indicator that calibration is starting
  setLEDColor(false, false, true);  // Blue LED during calibration
  
  // Wait a moment for the device to settle
  delay(500);
  
  // Variables for averaging and standard deviation
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumSqAccX = 0, sumSqAccY = 0, sumSqAccZ = 0;
  const int numSamples = 100;
  
  // Collect samples
  for (int i = 0; i < numSamples; i++) {
    float accX = imu.readFloatAccelX();
    float accY = imu.readFloatAccelY();
    float accZ = imu.readFloatAccelZ();
    
    // Accumulate for mean
    sumAccX += accX;
    sumAccY += accY;
    sumAccZ += accZ;
    
    // Accumulate for standard deviation
    sumSqAccX += sq(accX);
    sumSqAccY += sq(accY);
    sumSqAccZ += sq(accZ);
    
    delay(20);  // Sample at 50Hz
  }
  
  // Calculate mean values
  noiseFloorAccX = sumAccX / numSamples;
  noiseFloorAccY = sumAccY / numSamples;
  noiseFloorAccZ = sumAccZ / numSamples;
  
  // Calculate standard deviations
  float stdAccX = sqrt((sumSqAccX / numSamples) - sq(noiseFloorAccX));
  float stdAccY = sqrt((sumSqAccY / numSamples) - sq(noiseFloorAccY));
  float stdAccZ = sqrt((sumSqAccZ / numSamples) - sq(noiseFloorAccZ));
  
  // Mark as calibrated
  imuNoiseCalibrated = true;
  
  // Print calibration results
  Serial.println("Accelerometer noise floor calibration complete");
  Serial.println("Mean noise values:");
  Serial.print("X-axis: "); Serial.print(noiseFloorAccX, 4);
  Serial.print(" (±"); Serial.print(stdAccX, 4); Serial.println(" std)");
  
  Serial.print("Y-axis: "); Serial.print(noiseFloorAccY, 4);
  Serial.print(" (±"); Serial.print(stdAccY, 4); Serial.println(" std)");
  
  Serial.print("Z-axis: "); Serial.print(noiseFloorAccZ, 4);
  Serial.print(" (±"); Serial.print(stdAccZ, 4); Serial.println(" std)");
  
  // Update motion threshold based on noise floor
  float avgNoise = (stdAccX + stdAccY + stdAccZ) / 3.0;
  float suggestedThreshold = avgNoise * 8.0;  // 8 standard deviations
  
  Serial.print("Suggested motion threshold: ");
  Serial.println(suggestedThreshold, 4);
  Serial.print("Current threshold: ");
  Serial.println(MOTION_THRESHOLD, 4);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("XIAO nRF52840 Sense - Gesture Recognition");
  Serial.println("----------------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'train' - Enter training data collection mode");
  Serial.println("  'reset' - Reset to reference gesture recording mode");
  Serial.println("  'calibrate' - Recalibrate gyroscope and noise floor");
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
      
      // Show recording progress every 500ms
      unsigned long elapsedTime = currentTime - recordingStartTime;
      if (elapsedTime % 500 < SAMPLE_RATE_MS) {
        // Use LED color for progress indication only, don't print to serial during CSV recording
        updateProgressLED(elapsedTime);
      }
    }
    
    // Check if recording window has ended
    if (currentTime - recordingStartTime >= RECORDING_DURATION) {
      // Recording window ended
      isRecording = false;
      
      // Update status LED to indicate recording is complete
      setLEDColor(false, true, false); // Green - recording complete
      
      // Log recording completion
      Serial.println();  // Add a newline to separate from CSV data
      Serial.println("\nRECORDING FINISHED - " + String(currentGesture.sampleCount) + " samples collected");
      
      // Process the recorded data
      processGestureData(&currentGesture);
      
      // Different handling based on current mode
      if (inTrainingMode && repetitionCount < REQUIRED_REPETITIONS) {
        // More repetitions needed for current gesture
        repetitionCount++;
        
        Serial.print("\nReady for next repetition of gesture: ");
        Serial.print(gestureNames[currentGestureID]);
        Serial.print(" (");
        Serial.print(repetitionCount);
        Serial.print(" of ");
        Serial.print(REQUIRED_REPETITIONS);
        Serial.println(" completed)");
        
        // Return to waiting state (red LED)
        setLEDColor(true, false, false);
      } else if (inTrainingMode) {
        // We've collected enough of this gesture type
        repetitionCount = 0;
        currentGestureID = (currentGestureID + 1) % MAX_GESTURE_TYPES;
        
        if (currentGestureID == 0) {
          // We've cycled through all gesture types
          Serial.println("\n*** ALL GESTURE DATA COLLECTED! ***");
          Serial.println("Switching back to normal mode. Double-tap to record reference gestures.");
          inTrainingMode = false;
        } else {
          Serial.print("\nSwitching to gesture type: ");
          Serial.print(gestureNames[currentGestureID]);
          Serial.println("\nDouble-tap to start recording.");
        }
        
        // Return to normal state (red LED)
        setLEDColor(true, false, false);
      } else if (!inComparisonMode) {
        // Regular reference gesture recording
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
      } else {
        // Comparison mode
        // Process the recorded gesture
        processGestureData(&currentGesture);
        
        // Find the best match using DTW
        bestMatchIndex = findBestMatch(&currentGesture);
        
        // Display results
        if (bestMatchIndex >= 0 && bestMatchScore >= SIMILARITY_THRESHOLD) {
          // We have a match!
          Serial.print("\n*** MATCH FOUND: Reference gesture #");
          Serial.print(bestMatchIndex + 1);
          Serial.print(" (");
          Serial.print(bestMatchScore, 1);
          Serial.println("% similarity) ***");
          
          // Visual feedback - green LED for successful match
          setLEDColor(false, true, false);
          delay(2000);
        } else if (bestMatchIndex >= 0) {
          // Best match below threshold
          Serial.print("\nNo confident match. Closest: Reference #");
          Serial.print(bestMatchIndex + 1);
          Serial.print(" (");
          Serial.print(bestMatchScore, 1);
          Serial.println("% similarity)");
          
          // Visual feedback - yellow LED for weak match
          setLEDColor(true, true, false);
          delay(2000);
        } else {
          // No valid comparison
          Serial.println("\nNo valid reference gestures to compare with.");
          
          // Visual feedback - red flash for no match
          setLEDColor(true, false, false);
          delay(2000);
        }
        
        // Return to comparison mode waiting state (blue LED)
        setLEDColor(false, false, true);
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
    
    if (command == "train") {
      // Switch to training mode
      inTrainingMode = true;
      inComparisonMode = false;
      currentGestureID = 0;
      repetitionCount = 0;
      
      Serial.println("\n*** ENTERING TRAINING MODE ***");
      Serial.print("First gesture: ");
      Serial.println(gestureNames[currentGestureID]);
      Serial.println("Double-tap to start recording.");
      
      // Blink yellow LED to indicate training mode
      for (int i = 0; i < 3; i++) {
        setLEDColor(true, true, false); // Yellow
        delay(200);
        ledOff();
        delay(200);
      }
      setLEDColor(true, false, false); // Back to red (waiting state)
    }
    else if (command == "reset") {
      // Reset to reference recording mode
      inTrainingMode = false;
      inComparisonMode = false;
      currentReferenceIndex = 0;
      
      // Reset reference gestures
      for (int i = 0; i < 3; i++) {
        referenceGestures[i].isActive = false;
        referenceGestures[i].sampleCount = 0;
      }
      
      Serial.println("\n*** RESET TO REFERENCE RECORDING MODE ***");
      Serial.println("Double tap to start recording reference gesture #1");
      
      // Blink red LED to indicate reset
      for (int i = 0; i < 3; i++) {
        setLEDColor(true, false, false); // Red
        delay(200);
        ledOff();
        delay(200);
      }
      setLEDColor(true, false, false); // Back to red (waiting state)
    }
    else if (command == "calibrate") {
      // Perform full IMU calibration
      Serial.println("\n*** STARTING FULL IMU CALIBRATION ***");
      Serial.println("Place the device on a stable surface and don't move it");
      
      // Calibrate gyroscope
      calibrateGyroscope();
      
      // Calibrate accelerometer noise floor
      calibrateAccelerometerNoise();
      
      Serial.println("Calibration complete");
      setLEDColor(true, false, false); // Back to red (waiting state)
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