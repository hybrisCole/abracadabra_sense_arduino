#ifndef IMU_H
#define IMU_H

#include <LSM6DS3.h>
#include <Wire.h>
#include "../config/config.h"

class IMUSensor {
private:
  LSM6DS3 imu;

public:
  IMUSensor() : imu(I2C_MODE, IMU_I2C_ADDRESS) {}

  bool begin() {
    return (imu.begin() == 0);
  }

  float getAccelX() { return imu.readFloatAccelX(); }
  float getAccelY() { return imu.readFloatAccelY(); }
  float getAccelZ() { return imu.readFloatAccelZ(); }
  
  float getGyroX() { return imu.readFloatGyroX(); }
  float getGyroY() { return imu.readFloatGyroY(); }
  float getGyroZ() { return imu.readFloatGyroZ(); }
};

#endif