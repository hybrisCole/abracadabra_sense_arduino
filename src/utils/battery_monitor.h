#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
private:
  int batteryPin;
  float referenceVoltage;
  int numSamples;
  
  // LiPo typical voltage ranges
  const float MAX_VOLTAGE = 4.2;  // Fully charged 
  const float MIN_VOLTAGE = 3.3;  // Considered empty/safe discharge limit

public:
  BatteryMonitor(int pin = A0, float vRef = 3.3, int samples = 10) 
    : batteryPin(pin), referenceVoltage(vRef), numSamples(samples) {
    pinMode(batteryPin, INPUT);
  }

  float readVoltage(bool forceRead = false) {
    long sum = 0;
    
    // Take multiple readings and average them
    for (int i = 0; i < numSamples; i++) {
      sum += analogRead(batteryPin);
      delay(2);
    }
    
    // Calculate average
    float average = (float)sum / numSamples;
    
    // Always print raw analog reading - as requested
    Serial.print("Raw analog battery reading: ");
    Serial.println(average);
    
    // No battery detection (adjusted threshold for Expansion Base)
    if (!forceRead && average < 160) { // Changed from 100 to 160
      return 0.0;
    }
    
    // Map the raw ADC values to approximate battery voltage
    // Based on your readings of ~190-195 while charging a low battery
    float voltage = 0.0;
    
    if (average < 180) {
      // Critically low, <3.3V
      voltage = 3.2 + (average - 160) * 0.1 / 20;
    } else if (average < 200) {
      // Low, 3.3-3.5V
      voltage = 3.3 + (average - 180) * 0.2 / 20;
    } else if (average < 230) {
      // Medium-low, 3.5-3.7V
      voltage = 3.5 + (average - 200) * 0.2 / 30;
    } else if (average < 270) {
      // Medium, 3.7-3.9V 
      voltage = 3.7 + (average - 230) * 0.2 / 40;
    } else if (average < 320) {
      // Good, 3.9-4.1V 
      voltage = 3.9 + (average - 270) * 0.2 / 50;
    } else {
      // Full, 4.1-4.2V
      voltage = 4.1 + (average - 320) * 0.1 / 30;
    }
    
    return voltage;
  }
  
  int getPercentage(bool forceRead = false) {
    float voltage = readVoltage(forceRead);
    
    // Convert voltage to percentage
    int percentage = map(voltage * 100, MIN_VOLTAGE * 100, MAX_VOLTAGE * 100, 0, 100);
    
    // Constrain to 0-100 range
    percentage = constrain(percentage, 0, 100);
    
    return percentage;
  }
  
  String getStatus(bool forceRead = false) {
    int percentage = getPercentage(forceRead);
    
    if (percentage > 80) {
      return "High";
    } else if (percentage > 20) {
      return "Medium";
    } else {
      return "Low";
    }
  }
};

#endif