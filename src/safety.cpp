#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "ik.h"
#include "safety.h"
#include "servo_control.h"
#include "servo_manual.h"

void emergencyStop() {
  Serial.println("\n!!! EMERGENCY STOP !!!");
  isGaitRunning = false;

  JointAngles neutralAngles = computeIK(X_OFFSET, Y_OFFSET, Z_GROUND);
  if (neutralAngles.reachable) {
    for (int i = 0; i < 4; i++) {
      applyServos(neutralAngles, legs[i], i);
      delay(30);
    }
  }

  Serial.println("All servos at neutral. Safe to power down.");
}

void reportSaturationStats() {
  Serial.println("\n=== Saturation Report ===");
  Serial.print("Total events: ");
  Serial.println(satStats.totalSaturationEvents);

  const char* legNames[4] = {"FR", "FL", "BR", "BL"};  // Matches legs[] array order

  for (int i = 0; i < 4; i++) {
    if (satStats.shoulderSaturations[i] > 0 || satStats.hipSaturations[i] > 0 || satStats.kneeSaturations[i] > 0) {
      Serial.print(legNames[i]);
      Serial.print(" - Shoulder: ");
      Serial.print(satStats.shoulderSaturations[i]);
      Serial.print(" | Hip: ");
      Serial.print(satStats.hipSaturations[i]);
      Serial.print(" | Knee: ");
      Serial.println(satStats.kneeSaturations[i]);
    }
  }

  if (satStats.totalSaturationEvents == 0) {
    Serial.println("No saturation. Offsets tuned well.");
  } else {
    Serial.println("WARNING: Review offset values.");
  }
}

void initializeServos() {
  Serial.println("\nInitializing all servos to 90 degrees (with automatic offset compensation)...");
  initializationMode = true;

  Serial.println("Servo initialization:");
  const char* legNames[4] = {"FR", "FL", "BR", "BL"};  // Matches legs[] array order
  
  for (int i = 0; i < 4; i++) {
    Serial.print("  ");
    Serial.print(legNames[i]);
    Serial.println(" -> All servos to 90°");
    
    // Set all servos to 90 degrees (offsets automatically applied)
    // setServoAngleWithOffset(legs[i].shoulderCh, 90.0f);
    // setServoAngleWithOffset(legs[i].hipCh, 90.0f);
    // setServoAngleWithOffset(legs[i].kneeCh, 90.0f);
    setLegPosition(i, X_OFFSET, Y_OFFSET, Z_GROUND); // Use IK to set to neutral pose with offsets
    delay(50);
  }

  Serial.println("Settling...");
  delay(500);

  initializationMode = false;
  satStats.totalSaturationEvents = 0;
  for (int i = 0; i < 4; i++) {
    satStats.shoulderSaturations[i] = 0;
    satStats.hipSaturations[i] = 0;
    satStats.kneeSaturations[i] = 0;
  }
}
