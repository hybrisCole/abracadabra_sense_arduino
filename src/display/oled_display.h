#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>
#include "../sensors/imu.h"
#include "../utils/battery_monitor.h"

class OLEDDisplay {
private:
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;
  BatteryMonitor* batteryMonitor;
  IMUSensor* imuSensor;
  unsigned long lastUpdateTime = 0;
  const unsigned long updateInterval = 500; // Update display every 500ms
  
public:
  OLEDDisplay(BatteryMonitor* battery, IMUSensor* imu = nullptr) : 
    // Try both common addresses for SSD1306 OLED displays (0x3C and 0x3D)
    // Using software I2C with explicitly specified pins to ensure we're using the right ones
    u8x8(U8X8_PIN_NONE),
    batteryMonitor(battery),
    imuSensor(imu) {
    }
  
  bool begin() {
    // Initialize the OLED display with explicit I2C setup
    Wire.begin();
    // Try both common OLED addresses
    Serial.println("Trying to initialize OLED at address 0x3C...");
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
      Serial.println("Found OLED at address 0x3C");
    } else {
      Serial.println("No OLED at address 0x3C, trying 0x3D...");
      Wire.beginTransmission(0x3D);
      if (Wire.endTransmission() == 0) {
        Serial.println("Found OLED at address 0x3D");
      } else {
        Serial.println("No OLED found on I2C bus!");
      }
    }
    
    bool initSuccess = u8x8.begin();
    Serial.print("OLED initialization: ");
    Serial.println(initSuccess ? "SUCCESS" : "FAILED");
    
    if (!initSuccess) {
      return false;
    }
    
    u8x8.setPowerSave(0); // Make sure display is powered on
    u8x8.setFlipMode(1);   // Rotate display
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    
    // Draw simple text pattern
    u8x8.clearDisplay();
    u8x8.setCursor(0, 0);
    u8x8.print("OLED TEST");
    
    u8x8.setCursor(0, 2);
    u8x8.print("Line 1 --------");
    u8x8.setCursor(0, 3);
    u8x8.print("Line 2 --------");
    u8x8.setCursor(0, 4);
    u8x8.print("Line 3 --------");
    
    // Print complete display test message to serial
    Serial.println("Simple OLED test drawn");
    
    return true;
  }
  
  void update() {
    unsigned long currentTime = millis();
    
    // Update only at certain intervals to avoid display flicker
    if (currentTime - lastUpdateTime >= updateInterval) {
      lastUpdateTime = currentTime;
      
      // Clear display
      u8x8.clearDisplay();
      
      // Device info
      u8x8.setCursor(0, 0);
      u8x8.print("TEST DISPLAY!");
      
      // Battery info - use forceRead=true to always show actual voltage, even when very low
      float voltage = batteryMonitor->readVoltage(true);
      int percent = batteryMonitor->getPercentage(true);
      
      u8x8.setCursor(0, 2);
      u8x8.print("Batt: ");
      u8x8.print(voltage, 2);
      u8x8.print("V (");
      u8x8.print(percent);
      u8x8.print("%)");
      
      // Draw a box/pattern just to make something visible
      u8x8.setCursor(0, 4);
      u8x8.print("[==============]");
      u8x8.setCursor(0, 5);
      u8x8.print("[ DISPLAY TEST ]");
      u8x8.setCursor(0, 6);
      u8x8.print("[==============]");
      
      // Runtime
      unsigned long runTimeSeconds = currentTime / 1000;
      unsigned long minutes = (runTimeSeconds % 3600) / 60;
      unsigned long seconds = runTimeSeconds % 60;
      
      u8x8.setCursor(0, 7);
      u8x8.print("Runtime: ");
      if (minutes < 10) u8x8.print("0");
      u8x8.print(minutes);
      u8x8.print(":");
      if (seconds < 10) u8x8.print("0");
      u8x8.print(seconds);
      
      Serial.println("OLED display updated");
    }
  }
};

#endif