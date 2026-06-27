#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>

#include "telemetry.h"
#include "wifi_manager.h"
#include "webserver.h"
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
#include "stabilization.h"
#include "tasks.h"


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

  initHardwareLocks();
  pwm.begin();
  delay(50);  // Ensure PCA9685 has time to initialize
  pwm.setPWMFreq(FREQUENCY);
  delay(50);  // Wait for frequency to be applied
  
  // WiFi.softAP("Quadruped");
  // Serial.println(WiFi.softAPIP());
  connectWiFi();
  initWebUI();
  delay(100);  // Let WiFi stabilize

  initializeServos();
  delay(6000);  // Let servos move to initial position


  if (imuInit()) {
    Serial.println("IMU detected and initialized.");
  } else {
    Serial.println("IMU not available yet (running open-loop gait).");
  }


  stabilizationInit();
  startTasks();

  lastTime = millis();
  DEBUG_MODE = 0;
  robotMode = MODE_STAND;
  Serial.println("Ready. Type HELP for manual servo control commands.");
  printManualControlHelp();
}

void loop() {

   // Handle serial commands
  sendTelemetry();
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

//====================clock===============
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < CONTROL_DT) {
    return;
  }
  lastTime = now;
//====================clock end===============


  //=========================================leg line draw test code=========================================
  // //Test code for gait
  // float x0 = 0;
  // float y0 = 0;
  // float z0 = -18;

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
  RobotMode mode = robotMode;

  switch(mode)
{
    case MODE_MANUAL:
        // User is controlling legs manually.
        // Do not overwrite commands.
        break;

    case MODE_STAND:

        updateBodyCompensation();

        moveFoot(LEG_FR, X_OFFSET, Y_OFFSET, Z_GROUND);
        moveFoot(LEG_FL, X_OFFSET, Y_OFFSET, Z_GROUND);
        moveFoot(LEG_BR, X_OFFSET, Y_OFFSET, Z_GROUND);
        moveFoot(LEG_BL, X_OFFSET, Y_OFFSET, Z_GROUND);

        return;

    case MODE_GAIT:
    {
        float cycleDuration = GAIT_CYCLE_DURATION;
        if(cycleDuration < CONTROL_DT)
        {
            cycleDuration = CONTROL_DT;
        }

        float gaitPhase = phaseTime;
        gaitPhase += dt / cycleDuration;
        gaitPhase = fmod(gaitPhase, 1.0f);
        phaseTime = gaitPhase;

        updateGaitState(gaitPhase);
        updateSupportShift();
        updateBodyCompensation();

        for(int leg = 0; leg < 4; leg++)
        {
            squareGait(
                gaitState.phase[leg],
                legs[leg],
                leg
            );
        }

        static uint32_t lastPrint = 0;

        if(DEBUG_MODE && millis() - lastPrint > 500)
        {
            lastPrint = millis();

            Serial.print("phaseTime=");
            Serial.println(gaitPhase, 3);
        }
        break;
    }
}

}
