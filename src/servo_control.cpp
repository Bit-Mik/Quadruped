#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "servo_control.h"

void applyServos(const JointAngles &angles, LegConfig &leg, int legIndex) {
  if (!angles.reachable) {
    return;
  }

  float shoulderIK = angles.shoulder;
  float hipIK = angles.hip;
  float kneeIK = angles.knee;

  // Apply left/right side transformations
  if (leg.isLeftSide) {
    shoulderIK = -shoulderIK;  // Mirror shoulder for left side
    hipIK = -hipIK;             // Mirror hip for left side
  }
  if (!leg.isLeftSide) {
    kneeIK = -kneeIK;           // Mirror knee for right side
  }

  // Apply mechanical frame rotations
  shoulderIK += (leg.isLeftSide ? HIP_FRAME_ROTATION : -HIP_FRAME_ROTATION);
  hipIK += (leg.isLeftSide ? HIP_FRAME_ROTATION : -HIP_FRAME_ROTATION);

  // Apply mechanical offsets
  float shoulderServo = leg.shoulderMechOffset + shoulderIK;
  float hipServo = leg.hipMechOffset + hipIK;
  float kneeServo = leg.kneeMechOffset + kneeIK;

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
  pwm.setPWM(leg.shoulderCh, 0, PWM(shoulderServo));
  pwm.setPWM(leg.hipCh, 0, PWM(hipServo));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeServo));

  if (DEBUG_MODE) {
    const char* legNames[4] = {"BL", "FL", "BR", "FR"};
    
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
