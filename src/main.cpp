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
#define RECORDING_DURATION 10000   // Recording window duration in ms (10 seconds)

// Sampling configuration
#define SAMPLE_RATE_MS 20  // Sample every 20ms (approximately 50Hz)
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;

// Gesture storage constants
#define MAX_GESTURE_SAMPLES 500     // Maximum samples in a processed gesture (10sec @ 50Hz)

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

// Forward declarations
void processGestureData(GestureData* gesture);
void showRecordingProgress(unsigned long elapsedTime);
void updateProgressLED(unsigned long elapsedTime);

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
  
  Serial.println("\nData Processing:");
  Serial.println("* USING RAW SENSOR VALUES - No filtering or calibration applied");
  Serial.println("* Sample rate: 50Hz (typical for human gestures)");
  Serial.println("* No threshold or noise reduction applied");
  
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

// Process recorded gesture data - no filtering
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
  
  Serial.println("Gesture processing complete");
}

// Record and print current sensor data
void recordSensorData() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - recordingStartTime;
  
  // Read raw sensor data directly with no processing
  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  float gyroX = imu.readFloatGyroX();
  float gyroY = imu.readFloatGyroY();
  float gyroZ = imu.readFloatGyroZ();
  
  // Store in current gesture structure
  currentGesture.accX[sampleCount] = accX;
  currentGesture.accY[sampleCount] = accY;
  currentGesture.accZ[sampleCount] = accZ;
  currentGesture.gyroX[sampleCount] = gyroX;
  currentGesture.gyroY[sampleCount] = gyroY;
  currentGesture.gyroZ[sampleCount] = gyroZ;
  
  // Print in CSV format - simple output for reference gestures
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
  
  // We're in reference recording mode
  Serial.print("Recording REFERENCE gesture #");
  Serial.println(currentReferenceIndex + 1);
  
  // Start recording immediately
  Serial.println("NOW BEGIN YOUR GESTURE!");
  Serial.println("Recording started");
  setLEDColor(false, true, false); // Green - recording starts
  
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
  Serial.println("\nRECORDING STARTED - 10 second window");
  Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("XIAO nRF52840 Sense - Gesture Recording");
  Serial.println("----------------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'reset' - Reset to reference gesture recording mode");
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