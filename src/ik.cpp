#include <math.h>

#include "config.h"
#include "ik.h"

JointAngles computeIK(float targetX, float targetY) {
  JointAngles result;
  result.reachable = false;

  float distance = sqrt(targetX * targetX + targetY * targetY);

  if (distance >
          (UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - SINGULARITY_MARGIN) ||
      distance < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)) {
    return result;
  }

  float cosBeta =
      (targetX * targetX + targetY * targetY -
       UPPER_LEG_LENGTH * UPPER_LEG_LENGTH -
       LOWER_LEG_LENGTH * LOWER_LEG_LENGTH) /
      (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
  cosBeta = constrain(cosBeta, -1.0f, 1.0f);
  float kneeAngleRad = acos(cosBeta);

  float hipAngleRad =
      atan2(targetY, targetX) -
      atan2(LOWER_LEG_LENGTH * sin(kneeAngleRad),
            UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * cos(kneeAngleRad));

  float hipAngleDeg = hipAngleRad * 180.0f / PI;
  float kneeAngleDeg = kneeAngleRad * 180.0f / PI;

  result.hip = hipAngleDeg;
  result.knee = kneeAngleDeg;
  result.reachable = true;

  DEBUG_PRINT("IK - Hip deg: ");
  DEBUG_PRINT(hipAngleDeg);
  DEBUG_PRINT(" | Knee deg: ");
  DEBUG_PRINTLN(kneeAngleDeg);

  return result;
}
