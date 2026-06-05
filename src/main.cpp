#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "gait.h"
#include "globals.h"
#include "hardware.h"
#include "imu.h"
#include "safety.h"
#include "servo_manual.h"
#include "ik.h"
#include "fk.h"
#include "fun.h"

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
  delay(500);  // Wait for I2C to stabilize

  Serial.println("\n=== Quadruped Startup ===");

  if (!i2cDevicePresent(PCA9685_ADDR)) {
    Serial.println("ERROR: PCA9685 not found at 0x" + String(PCA9685_ADDR, HEX));
    while (1) {
    }
  }
  Serial.println("PCA9685 detected.");

  pwm.begin();
  delay(50);  // Ensure PCA9685 has time to initialize
  pwm.setPWMFreq(FREQUENCY);
  delay(50);  // Wait for frequency to be applied

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
  Serial.println("Ready. Type HELP for manual servo control commands.");
  printManualControlHelp();
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    static String command = "";
    
    // Echo incoming character
    Serial.write(c);
    
    if (c == '\n' || c == '\r') {
      if (command.length() > 0) {
        Serial.println();  // New line after command
        handleSerialCommand(command);
      }
      command = "";
    } else if (c >= 32 && c <= 126) {  // Printable ASCII only
      command += c;
    }
  }

  //=========================================leg line draw test code=========================================
  // //Test code for gait
  float x0 = 0;
  float y0 = 0;
  float z0 = -18;

  // for(float x = -5; x <= 5; x += 0.2)//for x axis test
  // {
  //   setLegPosition(0, x, y0, z0);
  //   delay(20);
  // }
  
//   for(float y = -5; y <= 5; y += 0.2)
// {
//     setLegPosition(0, x0, y, z0);
//     delay(20);
// }

// for(float z = -20; z <= -10; z += 0.2)
// {
//     setLegPosition(0, x0, y0, z);
//     delay(20);
// }
// setLegPosition(0, x0, y0, -18);
//=========================================leg line draw test code end=========================================

//=========================================FK test code=========================================
//   for(float z=-14; z>=-20; z-=1)
// {
//     JointAngles a = computeIK(0,0,z);

//     Serial.print("Z=");
//     Serial.print(z);

//     Serial.print(" S=");
//     Serial.print(a.shoulder);

//     Serial.print(" H=");
//     Serial.print(a.hip);

//     Serial.print(" K=");
//     Serial.println(a.knee);
// }
//=========================================FK test code end=========================================

  // danceSway();
  // happyDance();
  // twerk();
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

  squareGait(fmod(phaseTime + 0.00f, 1.0f), legs[LEG_BL], LEG_BL);
  squareGait(fmod(phaseTime + 0.25f, 1.0f), legs[LEG_FR], LEG_FR);
  squareGait(fmod(phaseTime + 0.50f, 1.0f), legs[LEG_FL], LEG_FL); 
  squareGait(fmod(phaseTime + 0.75f, 1.0f), legs[LEG_BR], LEG_BR);
}
