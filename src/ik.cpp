#include <math.h>

#include "config.h"
#include "ik.h"

// 3-Link IK solver for shoulder->hip->knee chain
// targetX, targetY: desired foot position in local leg frame
// gaitPhase: normalized phase (0.0-1.0) for synchronized shoulder movement
JointAngles computeIK(float targetX, float targetY, float gaitPhase) {
  JointAngles result;
  result.reachable = false;

  // Compute 3D distance from hip (center of rotation)
  float distance = sqrt(targetX * targetX + targetY * targetY);

  // Check reachability for 2-link hip-knee chain
  // Shoulder only affects horizontal offset, not reach
  if (distance > (UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - SINGULARITY_MARGIN) ||
      distance < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)) {
    return result;
  }

  // Calculate knee angle using law of cosines
  float cosBeta =
      (targetX * targetX + targetY * targetY -
       UPPER_LEG_LENGTH * UPPER_LEG_LENGTH -
       LOWER_LEG_LENGTH * LOWER_LEG_LENGTH) /
      (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
  cosBeta = constrain(cosBeta, -1.0f, 1.0f);
  float kneeAngleRad = acos(cosBeta);

  // Calculate hip angle
  float hipAngleRad =
      atan2(targetY, targetX) -
      atan2(LOWER_LEG_LENGTH * sin(kneeAngleRad),
            UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * cos(kneeAngleRad));

  float hipAngleDeg = hipAngleRad * 180.0f / PI;
  float kneeAngleDeg = kneeAngleRad * 180.0f / PI;

  // Shoulder angle: varies sinusoidally with gait phase for abduction/adduction
  // Synchronized across all legs at same phase value
  float shoulderAngleDeg = SHOULDER_AMPLITUDE * sin(2.0f * PI * gaitPhase);

  result.shoulder = shoulderAngleDeg;
  result.hip = hipAngleDeg;
  result.knee = kneeAngleDeg;
  result.reachable = true;

  DEBUG_PRINT("IK - Shoulder: ");
  DEBUG_PRINT(shoulderAngleDeg);
  DEBUG_PRINT(" | Hip: ");
  DEBUG_PRINT(hipAngleDeg);
  DEBUG_PRINT(" | Knee: ");
  DEBUG_PRINTLN(kneeAngleDeg);

  return result;
}
