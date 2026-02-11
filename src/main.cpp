#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "gait.h"
#include "globals.h"
#include "hardware.h"
#include "safety.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  Serial.println("\n=== Quadruped Startup ===");

  if (!i2cDevicePresent(PCA9685_ADDR)) {
    Serial.println("ERROR: PCA9685 not found at 0x" + String(PCA9685_ADDR, HEX));
    while (1) {
    }
  }
  Serial.println("PCA9685 detected.");

  pwm.begin();
  delay(10);
  pwm.setPWMFreq(FREQUENCY);

  initializeServos();

  lastTime = millis();
  DEBUG_MODE = true;
  isGaitRunning = true;
  Serial.println("Ready. Gait starting...");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      emergencyStop();
      reportSaturationStats();
      return;
    }
  }

  if (!isGaitRunning) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < CONTROL_DT) {
    return;
  }
  lastTime = now;

  phaseTime += dt / GAIT_CYCLE_DURATION;
  phaseTime = fmod(phaseTime, 1.0f);

  stepLeg(fmod(phaseTime + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BL], LEG_BL);
  stepLeg(fmod(phaseTime + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FL], LEG_FL);
  stepLeg(fmod(phaseTime + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FR], LEG_FR);
  stepLeg(fmod(phaseTime + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BR], LEG_BR);
}
