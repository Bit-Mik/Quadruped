#include <math.h>

#include "config.h"
#include "gait.h"
#include "ik.h"
#include "servo_control.h"

void stepLeg(float phase, float xOffset, float yGround, LegConfig &leg,
             int legIndex) {
  float targetX;
  float targetY;

  if (phase < 0.5f) {
    float phaseFraction = phase / 0.5f;
    targetX = xOffset + STEP_LENGTH * (phaseFraction - 0.5f);
    targetY = yGround + STEP_HEIGHT * sin(PI * phaseFraction);
  } else {
    float phaseFraction = (phase - 0.5f) / 0.5f;
    targetX = xOffset + STEP_LENGTH * (0.5f - phaseFraction);
    targetY = yGround;
  }

  JointAngles jointAngles = computeIK(targetX, targetY);
  if (jointAngles.reachable) {
    applyServos(jointAngles, leg, legIndex);
  } else {
    DEBUG_PRINTLN("Unreachable target");
  }
}
