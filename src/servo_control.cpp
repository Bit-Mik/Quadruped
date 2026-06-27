#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "servo_control.h"

float currentServoAngles[15] = {
    90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};



void applyServos(const JointAngles &angles, LegConfig &leg, int legIndex) {
  if (!angles.reachable) {
    return;
  }

  float shoulderIK = angles.shoulder;
  float hipIK = angles.hip;
  float kneeIK = angles.knee + 90.0f; // 90 Mechanical offset to match servo's neutral position

  if (!leg.isLeftSide) {
    kneeIK = kneeIK;           // Mirror knee for right side
    hipIK = - hipIK;           // Mirror hip for right side 
    // shoulderIK = 180 - shoulderIK; // Mirror shoulder for right side
  }else {
    kneeIK = 180 - kneeIK;           // Mirror knee for left side
    hipIK = hipIK;           // Mirror hip for left side
  }
  if(legIndex == 0 || legIndex == 3) { // FR and BL legs need shoulder mirroring due to mounting orientation
    shoulderIK = 180 - shoulderIK;
  }

  // Apply mechanical offsets
  float shoulderServo = shoulderIK;
  float hipServo = hipIK + 90.0f; // Add 90° to hip to match servo's neutral position
  float kneeServo = kneeIK;

  // Check saturation
  bool shoulderSat = (shoulderServo < MIN_SERVO_ANGLE || shoulderServo > MAX_SERVO_ANGLE);
  bool hipSat = (hipServo < MIN_SERVO_ANGLE || hipServo > MAX_SERVO_ANGLE);
  bool kneeSat = (kneeServo < MIN_SERVO_ANGLE || kneeServo > MAX_SERVO_ANGLE);

  if (!initializationMode && (shoulderSat || hipSat || kneeSat)) {
    satStats.totalSaturationEvents++;
    if (legIndex >= 0 && legIndex < 4) {
      if (shoulderSat) {
        satStats.shoulderSaturations[legIndex]++;
      }
      if (hipSat) {
        satStats.hipSaturations[legIndex]++;
      }
      if (kneeSat) {
        satStats.kneeSaturations[legIndex]++;
      }
    }
  }

  // Clamp to servo limits
  shoulderServo = constrain(shoulderServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  hipServo = constrain(hipServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  kneeServo = constrain(kneeServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  // Apply to servos
  setServoAngleWithOffset(leg.shoulderCh, shoulderServo);
  setServoAngleWithOffset(leg.hipCh, hipServo);
  setServoAngleWithOffset(leg.kneeCh, kneeServo);

  if (DEBUG_MODE) {
    const char* legNames[4] = {"FR", "FL", "BR", "BL"};
    
    if (legIndex >= 0 && legIndex < 4) {
      Serial.print(legNames[legIndex]);
    } else {
      Serial.print("??");
    }
    Serial.print(" | Shoulder IK: ");
    Serial.print(angles.shoulder);
    Serial.print(" -> Servo: ");
    Serial.print(shoulderServo);
    Serial.print(" | Hip IK: ");
    Serial.print(angles.hip);
    Serial.print(" -> Servo: ");
    Serial.print(hipServo);
    Serial.print(" | Knee IK: ");
    Serial.print(angles.knee);
    Serial.print(" -> Servo: ");
    Serial.println(kneeServo);
  }
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

  writeServoPWM(servoIndex, angle);

  Serial.print("Servo ");
  Serial.print(servoIndex);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println("°");
}
