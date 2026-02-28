#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "imu.h"

namespace {
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float GYRO_SCALE_RAD_PER_SEC =
    (250.0f / 32768.0f) * (PI / 180.0f);  // FS_SEL=0 (250 dps)
constexpr float RAD_TO_DEG = 180.0f / PI;

constexpr int GYRO_CALIBRATION_SAMPLES = 500;
constexpr bool CALIBRATE_GYRO_ON_STARTUP = true;

// Example offsets/scale for one calibrated MPU-6050.
// Recalibrate for your own module for best results.
float g_accelCalibration[6] = {265.0f, -80.0f, -700.0f, 0.994f, 1.000f, 1.014f};
float g_gyroOffsets[3] = {-499.5f, -17.7f, -82.0f};

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_kp = 30.0f;
float g_ki = 0.0f;

long g_gyroSum[3] = {0, 0, 0};
int g_calibrationCount = 0;
bool g_calibratingGyro = CALIBRATE_GYRO_ON_STARTUP;

ImuState g_imuState = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, false};

bool i2cWriteByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy,
                int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesRequested = 14;
  uint8_t bytesRead = Wire.requestFrom(MPU6050_ADDR, bytesRequested, true);
  if (bytesRead != bytesRequested) {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  (void)Wire.read();
  (void)Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

void mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz,
                  float dtSeconds) {
  float recipNorm;
  float vx;
  float vy;
  float vz;
  float ex;
  float ey;
  float ez;
  float qa;
  float qb;
  float qc;
  static float ix = 0.0f;
  static float iy = 0.0f;
  static float iz = 0.0f;

  float accelNormSq = ax * ax + ay * ay + az * az;
  if (accelNormSq > 0.0f) {
    recipNorm = 1.0f / sqrt(accelNormSq);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    vx = g_q[1] * g_q[3] - g_q[0] * g_q[2];
    vy = g_q[0] * g_q[1] + g_q[2] * g_q[3];
    vz = g_q[0] * g_q[0] - 0.5f + g_q[3] * g_q[3];

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (g_ki > 0.0f) {
      ix += g_ki * ex * dtSeconds;
      iy += g_ki * ey * dtSeconds;
      iz += g_ki * ez * dtSeconds;
      gx += ix;
      gy += iy;
      gz += iz;
    }

    gx += g_kp * ex;
    gy += g_kp * ey;
    gz += g_kp * ez;
  }

  float halfDt = 0.5f * dtSeconds;
  gx *= halfDt;
  gy *= halfDt;
  gz *= halfDt;
  qa = g_q[0];
  qb = g_q[1];
  qc = g_q[2];

  g_q[0] += (-qb * gx - qc * gy - g_q[3] * gz);
  g_q[1] += (qa * gx + qc * gz - g_q[3] * gy);
  g_q[2] += (qa * gy - qb * gz + g_q[3] * gx);
  g_q[3] += (qa * gz + qb * gy - qc * gx);

  recipNorm =
      1.0f / sqrt(g_q[0] * g_q[0] + g_q[1] * g_q[1] + g_q[2] * g_q[2] + g_q[3] * g_q[3]);
  g_q[0] *= recipNorm;
  g_q[1] *= recipNorm;
  g_q[2] *= recipNorm;
  g_q[3] *= recipNorm;
}

void updateEulerAndRates(float gxDps, float gyDps, float gzDps) {
  float rollRad = atan2((g_q[0] * g_q[1] + g_q[2] * g_q[3]),
                        0.5f - (g_q[1] * g_q[1] + g_q[2] * g_q[2]));
  float pitchRad = asin(2.0f * (g_q[0] * g_q[2] - g_q[1] * g_q[3]));
  float yawRad = -atan2((g_q[1] * g_q[2] + g_q[0] * g_q[3]),
                        0.5f - (g_q[2] * g_q[2] + g_q[3] * g_q[3]));

  g_imuState.rollDeg = rollRad * RAD_TO_DEG;
  g_imuState.pitchDeg = pitchRad * RAD_TO_DEG;
  g_imuState.yawDeg = yawRad * RAD_TO_DEG;
  if (g_imuState.yawDeg < 0.0f) {
    g_imuState.yawDeg += 360.0f;
  }

  g_imuState.rollRateDps = gxDps;
  g_imuState.pitchRateDps = gyDps;
  g_imuState.yawRateDps = gzDps;
}
}  // namespace

bool imuInit() {
  imuReset();

  Wire.beginTransmission(MPU6050_ADDR);
  bool present = (Wire.endTransmission() == 0);
  if (!present) {
    return false;
  }

  if (!i2cWriteByte(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00)) {
    return false;
  }

  delay(10);
  g_imuState.healthy = true;
  g_imuState.lastUpdateMs = millis();
  return true;
}

bool imuUpdate(float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return g_imuState.healthy;
  }

  int16_t axRaw;
  int16_t ayRaw;
  int16_t azRaw;
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;

  if (!readMpuRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    g_imuState.healthy = false;
    return false;
  }

  if (g_calibratingGyro) {
    g_gyroSum[0] += gxRaw;
    g_gyroSum[1] += gyRaw;
    g_gyroSum[2] += gzRaw;
    g_calibrationCount++;

    if (g_calibrationCount >= GYRO_CALIBRATION_SAMPLES) {
      g_gyroOffsets[0] = static_cast<float>(g_gyroSum[0]) / GYRO_CALIBRATION_SAMPLES;
      g_gyroOffsets[1] = static_cast<float>(g_gyroSum[1]) / GYRO_CALIBRATION_SAMPLES;
      g_gyroOffsets[2] = static_cast<float>(g_gyroSum[2]) / GYRO_CALIBRATION_SAMPLES;
      g_calibratingGyro = false;

      Serial.print("IMU gyro offsets: ");
      Serial.print(g_gyroOffsets[0]);
      Serial.print(", ");
      Serial.print(g_gyroOffsets[1]);
      Serial.print(", ");
      Serial.println(g_gyroOffsets[2]);
    }

    g_imuState.lastUpdateMs = millis();
    g_imuState.healthy = true;
    return true;
  }

  float ax = (static_cast<float>(axRaw) - g_accelCalibration[0]) * g_accelCalibration[3];
  float ay = (static_cast<float>(ayRaw) - g_accelCalibration[1]) * g_accelCalibration[4];
  float az = (static_cast<float>(azRaw) - g_accelCalibration[2]) * g_accelCalibration[5];

  float gxRad = (static_cast<float>(gxRaw) - g_gyroOffsets[0]) * GYRO_SCALE_RAD_PER_SEC;
  float gyRad = (static_cast<float>(gyRaw) - g_gyroOffsets[1]) * GYRO_SCALE_RAD_PER_SEC;
  float gzRad = (static_cast<float>(gzRaw) - g_gyroOffsets[2]) * GYRO_SCALE_RAD_PER_SEC;

  mahonyUpdate(ax, ay, az, gxRad, gyRad, gzRad, dtSeconds);

  float gxDps = gxRad * RAD_TO_DEG;
  float gyDps = gyRad * RAD_TO_DEG;
  float gzDps = gzRad * RAD_TO_DEG;
  updateEulerAndRates(gxDps, gyDps, gzDps);

  g_imuState.lastUpdateMs = millis();
  g_imuState.healthy = true;
  return true;
}

const ImuState &imuGetState() {
  return g_imuState;
}

void imuReset() {
  g_q[0] = 1.0f;
  g_q[1] = 0.0f;
  g_q[2] = 0.0f;
  g_q[3] = 0.0f;

  g_gyroSum[0] = 0;
  g_gyroSum[1] = 0;
  g_gyroSum[2] = 0;
  g_calibrationCount = 0;
  g_calibratingGyro = CALIBRATE_GYRO_ON_STARTUP;

  g_imuState.rollDeg = 0.0f;
  g_imuState.pitchDeg = 0.0f;
  g_imuState.yawDeg = 0.0f;
  g_imuState.rollRateDps = 0.0f;
  g_imuState.pitchRateDps = 0.0f;
  g_imuState.yawRateDps = 0.0f;
  g_imuState.lastUpdateMs = 0;
  g_imuState.healthy = false;
}
