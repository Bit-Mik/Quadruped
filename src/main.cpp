#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "gait.h"
#include "globals.h"
#include "hardware.h"
#include "imu.h"
#include "safety.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// IMU update task for FreeRTOS (ESP32 only)
static void imuTaskFunc(void *pvParameters) {
  (void)pvParameters;
  const TickType_t period = pdMS_TO_TICKS((int)(CONTROL_DT * 1000));
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    imuUpdate(CONTROL_DT);
    vTaskDelayUntil(&lastWake, period);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  // Use explicit SDA/SCL pins on ESP32 WROOM (default: 21=SDA, 22=SCL)
#ifdef ESP32
  Wire.begin(21, 22);
#else
  Wire.begin();
#endif
  delay(100);

  Serial.println("\n=== Quadruped Startup ===");

  // if (!i2cDevicePresent(PCA9685_ADDR)) {
  //   Serial.println("ERROR: PCA9685 not found at 0x" + String(PCA9685_ADDR, HEX));
  //   while (1) {
  //   }
  // }
  Serial.println("PCA9685 detected.");

  pwm.begin();
  delay(10);
  pwm.setPWMFreq(FREQUENCY);

  initializeServos();

  if (imuInit()) {
    Serial.println("IMU detected and initialized.");
  } else {
    Serial.println("IMU not available yet (running open-loop gait).");
  }

#ifdef ESP32
  // Create a FreeRTOS task to drive IMU updates at CONTROL_DT interval
  BaseType_t r = xTaskCreate(imuTaskFunc, "IMU", 4096, NULL, 2, NULL);
  if (r != pdPASS) {
    Serial.println("WARNING: Failed to create IMU task");
  }
#endif

  lastTime = millis();
  DEBUG_MODE = true;
  isGaitRunning = false;
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

  // IMU update handled in FreeRTOS task on ESP32; keep calling on AVR/UNO.
#ifndef ESP32
  imuUpdate(dt);
#endif

  phaseTime += dt / GAIT_CYCLE_DURATION;
  phaseTime = fmod(phaseTime, 1.0f);

  stepLeg(fmod(phaseTime + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BL], LEG_BL);
  stepLeg(fmod(phaseTime + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FL], LEG_FL);
  stepLeg(fmod(phaseTime + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FR], LEG_FR);
  stepLeg(fmod(phaseTime + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BR], LEG_BR);
}
