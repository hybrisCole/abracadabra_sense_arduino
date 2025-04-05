#ifndef JSON_SERIALIZER_H
#define JSON_SERIALIZER_H

#include <ArduinoJson.h>
#include "../config/config.h"
#include "../sensors/imu.h"
#include "../utils/battery_monitor.h"

class JSONSerializer {
private:
  StaticJsonDocument<JSON_DOCUMENT_SIZE> doc;

public:
  JSONSerializer() {}

  void serializeIMUData(IMUSensor& imu, unsigned long timestamp) {
    doc.clear();
    doc["timestamp"] = timestamp;
    
    JsonObject imuObj = doc.createNestedObject("imu");
    imuObj["accel_x"] = imu.getAccelX();
    imuObj["accel_y"] = imu.getAccelY();
    imuObj["accel_z"] = imu.getAccelZ();
    imuObj["gyro_x"] = imu.getGyroX();
    imuObj["gyro_y"] = imu.getGyroY();
    imuObj["gyro_z"] = imu.getGyroZ();
  }

  void serializeWithBattery(IMUSensor& imu, BatteryMonitor& battery, unsigned long timestamp) {
    // First add the IMU data
    serializeIMUData(imu, timestamp);
    
    // Add battery information
    JsonObject batteryObj = doc.createNestedObject("battery");
    batteryObj["voltage"] = battery.readVoltage();
    batteryObj["percent"] = battery.getPercentage();
    batteryObj["status"] = battery.getStatus();
  }

  void sendToSerial() {
    serializeJson(doc, Serial);
    Serial.println();
  }

  void sendStatusMessage(const char* status, const char* message) {
    doc.clear();
    doc["status"] = status;
    doc["message"] = message;
    serializeJson(doc, Serial);
    Serial.println();
  }
};

#endif