#include <math.h>
#include <Wire.h>

#include "config.h"
#include "hardware.h"

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
