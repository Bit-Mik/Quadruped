#include <Arduino.h>
#include "servo_manual.h"
#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "ik.h"
#include "servo_control.h"

// Track current servo angles (0-14 for 15 total servos)
static float currentServoAngles[15] = {
    90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};

void printManualControlHelp() {
  Serial.println("\n=== Manual Servo Control Help ===");
  Serial.println("Channel mapping: FR(0,1,2) FL(4,5,6) BR(8,9,10) BL(12,13,14)");
  Serial.println("  0=FR Shoulder  1=FR Hip     2=FR Knee");
  Serial.println("  4=FL Shoulder  5=FL Hip     6=FL Knee");
  Serial.println("  8=BR Shoulder  9=BR Hip    10=BR Knee");
  Serial.println(" 12=BL Shoulder 13=BL Hip    14=BL Knee");
  Serial.println("\nDirect Servo Commands:");
  Serial.println("  S<n>=<angle>   : Set servo N to angle (0-180°). Ex: S0=90");
  Serial.println("  S<n>?          : Get current angle of servo N. Ex: S0?");
  Serial.println("  ALL=<angle>    : Set ALL servos to angle (0-180°)");
  Serial.println("  HOME           : Set all servos to 90°");
  Serial.println("  STATUS         : Show all current servo angles");
  Serial.println("\nLeg IK Commands (Move leg to coordinates):");
  Serial.println("  FR=<x>,<y>,<z> : Move Front-Right leg to X,Y,Z (cm). Ex: FR=0,0,-19");
  Serial.println("  FL=<x>,<y>,<z> : Move Front-Left leg to X,Y,Z (cm)");
  Serial.println("  BR=<x>,<y>,<z> : Move Back-Right leg to X,Y,Z (cm)");
  Serial.println("  BL=<x>,<y>,<z> : Move Back-Left leg to X,Y,Z (cm)");
  Serial.println("\nGait Commands:");
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

  // Apply to PCA9685 with automatic offset compensation
  setServoAngleWithOffset(servoIndex, angle);

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

void setLegPosition(int legIndex, float targetX, float targetY, float targetZ) {
  if (legIndex < 0 || legIndex > 3) {
    Serial.println("ERROR: Invalid leg index");
    return;
  }

  // Compute IK for the target position
  JointAngles angles = computeIK(targetX, targetY, targetZ, 0.0f);
  
  if (!angles.reachable) {
    Serial.println("ERROR: Target position unreachable");
    return;
  }

  // Apply the angles to the leg
  applyServos(angles, legs[legIndex], legIndex);

  const char* legNames[4] = {"FR", "FL", "BR", "BL"};  // Matches legs[] array order
  Serial.print(legNames[legIndex]);
  Serial.print(" moved to X=");
  Serial.print(targetX);
  Serial.print(" cm, Y=");
  Serial.print(targetY);
  Serial.print(" cm, Z=");
  Serial.print(targetZ);
  Serial.println(" cm");
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

  // Parse leg IK commands: FR=x,y,z or FL=x,y,z or BR=x,y,z or BL=x,y,z
  if (command.startsWith("FR=") || command.startsWith("FL=") || 
      command.startsWith("BR=") || command.startsWith("BL=")) {
    
    String legName = command.substring(0, 2);
    String coordStr = command.substring(3);
    
    // Parse X,Y,Z coordinates
    int comma1 = coordStr.indexOf(',');
    int comma2 = coordStr.indexOf(',', comma1 + 1);
    
    if (comma1 < 0 || comma2 < 0) {
      Serial.println("ERROR: Use LEG=x,y,z format. Ex: FR=0,0,-19");
      return;
    }
    
    String xStr = coordStr.substring(0, comma1);
    String yStr = coordStr.substring(comma1 + 1, comma2);
    String zStr = coordStr.substring(comma2 + 1);
    
    xStr.trim();
    yStr.trim();
    zStr.trim();
    
    float targetX = xStr.toFloat();
    float targetY = yStr.toFloat();
    float targetZ = zStr.toFloat();
    
    int legIndex = -1;
    if (legName == "FR") legIndex = LEG_FR;
    else if (legName == "FL") legIndex = LEG_FL;
    else if (legName == "BR") legIndex = LEG_BR;
    else if (legName == "BL") legIndex = LEG_BL;
    
    if (legIndex >= 0) {
      setLegPosition(legIndex, targetX, targetY, targetZ);
    }
    return;
  }

  Serial.println("ERROR: Unknown command. Type HELP for commands.");
}
