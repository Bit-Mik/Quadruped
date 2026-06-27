#include <math.h>
#include <Wire.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t pwmMutex = nullptr;
#endif

void initHardwareLocks()
{
#ifdef ESP32
  if (pwmMutex == nullptr) {
    pwmMutex = xSemaphoreCreateMutex();
    if (pwmMutex == nullptr) {
      Serial.println("WARNING: Failed to create PWM mutex");
    }
  }
#endif
}

bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int PWM(float angle) {
  float pulse = 0.5f + 2.0f * angle / 180.0f;
  int ticks = round(pulse * 4096.0f / PERIOD_MS);
  ticks = constrain(ticks, 0, 4095);
  return ticks;
}

void writeServoPWM(int servoIndex, float angle)
{
  if (servoIndex < 0 || servoIndex > 14) {
    return;
  }

  float finalAngle = constrain(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  int ticks = PWM(finalAngle);

#ifdef ESP32
  if (pwmMutex != nullptr) {
    xSemaphoreTake(pwmMutex, portMAX_DELAY);
    pwm.setPWM(servoIndex, 0, ticks);
    xSemaphoreGive(pwmMutex);
    return;
  }
#endif

  pwm.setPWM(servoIndex, 0, ticks);
}

void setServoAngleWithOffset(int servoIndex, float angle) {
  if (servoIndex < 0 || servoIndex > 14) {
    return;
  }

  // Map servo index to leg and joint type
  // FR: 0,1,2 | FL: 4,5,6 | BR: 8,9,10 | BL: 12,13,14
  int legIndex = servoIndex / 4;  // Which leg (0=FR, 1=FL, 2=BR, 3=BL)
  int jointType = servoIndex % 4; // 0=shoulder, 1=hip, 2=knee, 3=unused
  
  float offset = 0.0f;
  if (jointType == 0) {
    offset = legs[legIndex].sOffset;  // Shoulder offset
  } else if (jointType == 1) {
    offset = legs[legIndex].hOffset;  // Hip offset
  } else if (jointType == 2) {
    offset = legs[legIndex].kOffset;  // Knee offset
  }
  
  float finalAngle = angle + offset;
  writeServoPWM(servoIndex, finalAngle);
}
