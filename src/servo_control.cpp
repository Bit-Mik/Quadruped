#include <Arduino.h>

#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "servo_control.h"

void applyServos(const JointAngles &angles, LegConfig &leg, int legIndex) {
  if (!angles.reachable) {
    return;
  }

  float hipIK = angles.hip;
  float kneeIK = angles.knee;

  if (leg.isLeftSide) {
    hipIK = -hipIK;
  }
  if (!leg.isLeftSide) {
    kneeIK = -kneeIK;
  }
  hipIK += (leg.isLeftSide ? HIP_FRAME_ROTATION : -HIP_FRAME_ROTATION);

  float hipServo = leg.hipMechOffset + hipIK;
  float kneeServo = leg.kneeMechOffset + kneeIK;

  bool hipSat = (hipServo < MIN_SERVO_ANGLE || hipServo > MAX_SERVO_ANGLE);
  bool kneeSat = (kneeServo < MIN_SERVO_ANGLE || kneeServo > MAX_SERVO_ANGLE);

  if (!initializationMode && (hipSat || kneeSat)) {
    satStats.totalSaturationEvents++;
    if (legIndex >= 0 && legIndex < 4) {
      if (hipSat) {
        satStats.hipSaturations[legIndex]++;
      }
      if (kneeSat) {
        satStats.kneeSaturations[legIndex]++;
      }
    }
  }

  hipServo = constrain(hipServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  kneeServo = constrain(kneeServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  pwm.setPWM(leg.hipCh, 0, PWM(hipServo));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeServo));

  if (DEBUG_MODE) {
    Serial.print("Leg ");
    Serial.print(legIndex);
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
