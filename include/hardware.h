#pragma once

#include <Arduino.h>

bool i2cDevicePresent(uint8_t address);
int PWM(float angle);
void initHardwareLocks();
void writeServoPWM(int servoIndex, float angle);
void setServoAngleWithOffset(int servoIndex, float angle);
