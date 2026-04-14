#include <Arduino.h>
#include "servo_manual.h"
#include "config.h"
#include "globals.h"
#include "hardware.h"

// Track current servo angles (0-14 for 15 total servos)
static float currentServoAngles[15] = {
    90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};

void printManualControlHelp() {
  Serial.println("\n=== Manual Servo Control Help ===");
  Serial.println("Channel mapping: BL(0,1,2) BR(4,5,6) FL(8,9,10) FR(12,13,14)");
  Serial.println("  0=BL Shoulder  1=BL Hip     2=BL Knee");
  Serial.println("  4=BR Shoulder  5=BR Hip     6=BR Knee");
  Serial.println("  8=FL Shoulder  9=FL Hip    10=FL Knee");
  Serial.println(" 12=FR Shoulder 13=FR Hip    14=FR Knee");
  Serial.println("Commands:");
  Serial.println("  S<n>=<angle>   : Set servo N to angle (0-180°). Ex: S0=90");
  Serial.println("  S<n>?          : Get current angle of servo N. Ex: S0?");
  Serial.println("  ALL=<angle>    : Set ALL servos to angle (0-180°)");
  Serial.println("  HOME           : Set all servos to 90°");
  Serial.println("  STATUS         : Show all current servo angles");
  Serial.println("  G              : Start gait");
  Serial.println("  STOP           : Stop gait / Emergency stop");
  Serial.println("  HELP           : Show this help message");
  Serial.println("====================================\n");
}

void setServoManual(int servoIndex, float angle) {
  if (servoIndex < 0 || servoIndex > 14) {
    Serial.print("ERROR: Invalid servo index ");
    Serial.print(servoIndex);
    Serial.println(" (valid: 0-14)");
    return;
  }

  angle = constrain(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  currentServoAngles[servoIndex] = angle;

  // Apply to PCA9685
  pwm.setPWM(servoIndex, 0, PWM(angle));

  Serial.print("Servo ");
  Serial.print(servoIndex);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println("°");
}

float getServoAngle(int servoIndex) {
  if (servoIndex < 0 || servoIndex > 14) {
    return -1.0f;
  }
  return currentServoAngles[servoIndex];
}

void handleSerialCommand(String command) {
  command.trim();
  command.toUpperCase();

  // HELP command
  if (command == "HELP") {
    printManualControlHelp();
    return;
  }

  // START/START GAIT
  if (command == "G" || command == "START" || command == "GAIT") {
    isGaitRunning = true;
    Serial.println("Gait started.");
    return;
  }

  // STOP/EMERGENCY STOP
  if (command == "STOP" || command == "S") {
    isGaitRunning = false;
    Serial.println("Gait stopped.");
    return;
  }

  // HOME - Set all to 90°
  if (command == "HOME") {
    for (int i = 0; i < 15; i++) {
      setServoManual(i, 90.0f);
    }
    return;
  }

  // STATUS - Show all servo states
  if (command == "STATUS") {
    Serial.println("\n=== Current Servo Angles ===");
    for (int i = 0; i < 15; i++) {
      Serial.print("S");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(currentServoAngles[i]);
      Serial.println("°");
    }
    Serial.println("=============================\n");
    return;
  }

  // Parse S<n>=<angle> or S<n>?
  if (command.startsWith("S")) {
    // Extract servo number
    int eqPos = command.indexOf('=');
    int qPos = command.indexOf('?');

    if (eqPos >= 0 && eqPos > 0 && eqPos < command.length() - 1) {
      // S<n>=<angle> format
      String numStr = command.substring(1, eqPos);
      String angleStr = command.substring(eqPos + 1);

      // Validate and convert
      numStr.trim();
      angleStr.trim();

      // Check if number is valid
      bool numValid = true;
      for (int i = 0; i < numStr.length(); i++) {
        if (numStr[i] < '0' || numStr[i] > '9') {
          numValid = false;
          break;
        }
      }

      if (!numValid || numStr.length() == 0) {
        Serial.println("ERROR: Invalid servo number format");
        return;
      }

      int servoIdx = numStr.toInt();
      float angle = angleStr.toFloat();

      setServoManual(servoIdx, angle);
    } else if (qPos >= 0 && qPos > 0) {
      // S<n>? format - query servo angle
      String numStr = command.substring(1, qPos);
      numStr.trim();

      bool numValid = true;
      for (int i = 0; i < numStr.length(); i++) {
        if (numStr[i] < '0' || numStr[i] > '9') {
          numValid = false;
          break;
        }
      }

      if (!numValid || numStr.length() == 0) {
        Serial.println("ERROR: Invalid servo number format");
        return;
      }

      int servoIdx = numStr.toInt();

      if (servoIdx < 0 || servoIdx > 14) {
        Serial.print("ERROR: Invalid servo index ");
        Serial.println(servoIdx);
        return;
      }

      Serial.print("Servo ");
      Serial.print(servoIdx);
      Serial.print(" angle: ");
      Serial.print(currentServoAngles[servoIdx]);
      Serial.println("°");
    } else {
      Serial.println("ERROR: Use S<n>=<angle> or S<n>?");
    }
    return;
  }

  // Parse ALL=<angle>
  if (command.startsWith("ALL=")) {
    String angleStr = command.substring(4);
    float angle = angleStr.toFloat();

    if (angle < 0 || angle > 180) {
      Serial.println("ERROR: Angle must be 0-180");
      return;
    }

    Serial.print("Setting all servos to ");
    Serial.print(angle);
    Serial.println("°...");

    for (int i = 0; i < 15; i++) {
      setServoManual(i, angle);
    }
    return;
  }

  Serial.println("ERROR: Unknown command. Type HELP for commands.");
}
