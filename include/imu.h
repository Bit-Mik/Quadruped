#pragma once

#include <Arduino.h>

struct ImuRawSample {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

struct ImuState {
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float rollRateDps;
  float pitchRateDps;
  float yawRateDps;
  unsigned long lastUpdateMs;
  bool healthy;
};

bool imuInit();
bool imuUpdate(float dtSeconds);
const ImuState &imuGetState();
void imuReset();
void imuLoadCalibration();   // Load gyro offsets from NVS (ESP32) or use defaults
void imuSaveCalibration();   // Save gyro offsets to NVS (ESP32 only)

