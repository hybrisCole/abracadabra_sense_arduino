#include <Arduino.h>
#include "config/config.h"
#include "sensors/imu.h"
#include "utils/led_manager.h"
#include "communication/json_serializer.h"

IMUSensor imuSensor;
LEDManager led(LED_BUILTIN);
JSONSerializer jsonSerializer;

unsigned long previousMillis = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  delay(SERIAL_INIT_DELAY);
  
  Serial.println("XIAO nRF52840 Sense - JSON Output Test");

  if (!imuSensor.begin()) {
    jsonSerializer.sendStatusMessage("error", "IMU device error");
  } else {
    jsonSerializer.sendStatusMessage("ready", "IMU initialized");
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= SENSOR_READ_INTERVAL) {
    previousMillis = currentMillis;
    
    // Toggle LED to indicate activity
    led.toggle();
    
    // Serialize and send sensor data
    jsonSerializer.serializeIMUData(imuSensor, currentMillis);
    jsonSerializer.sendToSerial();
  }
}