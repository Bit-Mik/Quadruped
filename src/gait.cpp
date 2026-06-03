#include <math.h>

#include "config.h"
#include "gait.h"
#include "ik.h"
#include "servo_control.h"

void stepLeg(float phase, float xOffset, float zGround, LegConfig &leg,
             int legIndex) {
  float targetX;
  float targetZ;

  if (phase < 0.5f) {
    float phaseFraction = phase / 0.5f;
    targetX = xOffset + STEP_LENGTH * (phaseFraction - 0.5f);
    targetZ = zGround + STEP_HEIGHT * sin(PI * phaseFraction);
  } else {
    float phaseFraction = (phase - 0.5f) / 0.5f;
    targetX = xOffset + STEP_LENGTH * (0.5f - phaseFraction);
    targetZ = zGround;
  }

  // Pass the phase to IK for shoulder movement
  JointAngles jointAngles = computeIK(targetX, targetZ, phase);
  if (jointAngles.reachable) {
    applyServos(jointAngles, leg, legIndex);
  } else {
    DEBUG_PRINTLN("Unreachable target");
  }
}
