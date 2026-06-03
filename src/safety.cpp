#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "ik.h"
#include "safety.h"
#include "servo_control.h"

void emergencyStop() {
  Serial.println("\n!!! EMERGENCY STOP !!!");
  isGaitRunning = false;

  JointAngles neutralAngles = computeIK(X_OFFSET, Z_GROUND, 0.0f);
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

  const char* legNames[4] = {"BL", "FL", "BR", "FR"};  // Back-Left, Front-Left, Back-Right, Front-Right

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
  Serial.println("\nInitializing all servos to 90 degrees (with offset compensation)...");
  initializationMode = true;

  Serial.println("Servo initialization:");
  const char* legNames[4] = {"BL", "FL", "BR", "FR"};
  
  for (int i = 0; i < 4; i++) {
    // Apply offsets to achieve calibrated 90-degree position
    float shoulderAngle = 90.0f + legs[i].sOffset;
    float hipAngle = 90.0f + legs[i].hOffset;
    float kneeAngle = 90.0f + legs[i].kOffset;
    
    // Clamp to servo limits
    shoulderAngle = constrain(shoulderAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    hipAngle = constrain(hipAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    kneeAngle = constrain(kneeAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    
    Serial.print("  ");
    Serial.print(legNames[i]);
    Serial.print(" Shoulder=");
    Serial.print(shoulderAngle);
    Serial.print("° Hip=");
    Serial.print(hipAngle);
    Serial.print("° Knee=");
    Serial.print(kneeAngle);
    Serial.println("°");
    
    // Set servos with offset compensation
    pwm.setPWM(legs[i].shoulderCh, 0, PWM(shoulderAngle));
    pwm.setPWM(legs[i].hipCh, 0, PWM(hipAngle));
    pwm.setPWM(legs[i].kneeCh, 0, PWM(kneeAngle));
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
