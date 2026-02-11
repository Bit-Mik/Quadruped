#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "ik.h"
#include "safety.h"
#include "servo_control.h"

void emergencyStop() {
  Serial.println("\n!!! EMERGENCY STOP !!!");
  isGaitRunning = false;

  JointAngles neutralAngles = computeIK(X_OFFSET, Y_GROUND);
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

  for (int i = 0; i < 4; i++) {
    if (satStats.hipSaturations[i] > 0 || satStats.kneeSaturations[i] > 0) {
      Serial.print("Leg ");
      Serial.print(i);
      Serial.print(" - Hip: ");
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
  Serial.println("\nInitializing servos to standing posture...");
  initializationMode = true;

  JointAngles neutralAngles = computeIK(X_OFFSET, Y_GROUND);
  if (!neutralAngles.reachable) {
    Serial.println("ERROR: Standing posture unreachable!");
    while (1) {
    }
  }

  Serial.print("Neutral IK: Hip=");
  Serial.print(neutralAngles.hip);
  Serial.print(" deg Knee=");
  Serial.print(neutralAngles.knee);
  Serial.println(" deg");

  Serial.println("Servo initialization:");
  for (int i = 0; i < 4; i++) {
    float hipBeforeClamp =
        legs[i].hipMechOffset +
        (legs[i].isLeftSide ? -neutralAngles.hip : neutralAngles.hip);
    float kneeBeforeClamp = legs[i].kneeMechOffset - neutralAngles.knee;
    float hipClamped = constrain(hipBeforeClamp, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    float kneeClamped =
        constrain(kneeBeforeClamp, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

    Serial.print("  Leg ");
    Serial.print(i);
    Serial.print(" Hip=");
    Serial.print(hipClamped);
    Serial.print(" deg");
    if (hipBeforeClamp != hipClamped) {
      Serial.print("[SAT]");
    }
    Serial.print(" Knee=");
    Serial.print(kneeClamped);
    Serial.print(" deg");
    if (kneeBeforeClamp != kneeClamped) {
      Serial.print("[SAT]");
    }
    Serial.println();

    applyServos(neutralAngles, legs[i], i);
    delay(50);
  }

  Serial.println("Settling...");
  delay(500);

  initializationMode = false;
  satStats.totalSaturationEvents = 0;
  for (int i = 0; i < 4; i++) {
    satStats.hipSaturations[i] = 0;
    satStats.kneeSaturations[i] = 0;
  }
}
