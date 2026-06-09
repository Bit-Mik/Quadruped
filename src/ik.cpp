#include <math.h>

#include "config.h"
#include "ik.h"

// 3-Link IK solver for shoulder->hip->knee chain
//
// Coordinate system:
//   X: forward/backward (positive = forward)
//   Y: lateral (left/right from shoulder, positive = outward)
//   Z: vertical (negative = down towards ground)
//
// The solver works in two stages:
//   1. Shoulder joint: positions the leg in Y-Z plane to reach target
//   2. Hip-Knee 2-DOF chain: reaches target in X-Z plane with hip and knee
//
// Parameters:
//   targetX: forward/backward distance from hip (cm)
//   targetY: lateral distance from shoulder (cm)
//   targetZ: vertical position, ground reference (cm, typically -19)
//   gaitPhase: normalized cycle phase (0.0-1.0) for shoulder oscillation
JointAngles computeIK(float X, float Y, float Z) {
  JointAngles result;
  result.reachable = false;

  // ===== STAGE 1: REACHABILITY CHECK =====
  
  // Compute distances in the two work planes
  // Y-Z plane: shoulder and hip-knee reach in lateral-vertical direction
  float yzPlanarReach = sqrt(Y * Y + Z * Z);
  // X-Z plane: hip-knee reach in forward-vertical direction
  float xzPlanarReach = sqrt(X * X + Z * Z);
  if(xzPlanarReach > UPPER_LEG_LENGTH + LOWER_LEG_LENGTH)
{
    return result;
}
if(xzPlanarReach < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH))
{
    return result;
}

  // Account for shoulder servo mechanical offset
  // The hip-knee chain must reach from shoulder pivot to foot, minus shoulder offset
  float shoulderToFootDist = sqrt(yzPlanarReach * yzPlanarReach - SHOULDER_WIDTH * SHOULDER_WIDTH) - SHOULDER_LENGTH;
  float hipKneeReachRequired = shoulderToFootDist - SHOULDER_LENGTH;
  
  // Validate that hip-knee chain can reach the required distance
  if (hipKneeReachRequired > (UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - SINGULARITY_MARGIN) ||
      hipKneeReachRequired < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)) {
    return result;  // Position unreachable
  }

  // ===== STAGE 2: SHOULDER ANGLE (Y-Z PLANE) =====
  
  // Angle from vertical (Z-axis) to foot in lateral plane
  float beta1 = atan2(Y,-Z);
  
  // Shoulder mechanical offset angle in Y-Z plane
  float beta2 = atan2(SHOULDER_WIDTH, yzPlanarReach);
  
  // Shoulder servo angle = 90° minus combined angles
  // float theta1 = PI/2 - (beta1 + beta2); // Shoulder angle in Y-Z plane
  float theta1 = beta1 + PI/2; // Shoulder angle in Y-Z plane (alternative convention)
  // ===== STAGE 3: HIP-KNEE ANGLES (X-Z PLANE) =====
  
  // Angle from vertical (Z-axis) to foot in forward plane

  float xzPlaneAngle = atan2(X, -Z);
  
  // Law of cosines: interior angle of triangle (upper_leg, lower_leg, xzReach)
  // This is the angle at the knee joint
  float alpha1 = (UPPER_LEG_LENGTH * UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * LOWER_LEG_LENGTH - xzPlanarReach * xzPlanarReach) /
                     (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
  alpha1 = acos(constrain(alpha1, -1.0f, 1.0f)); // Clamp for safety
  float theta3 = alpha1 - PI; // Knee angle (supplementary to interior angle)
  
  // Law of cosines: angle between upper leg and vertical line to foot
  // This gives the hip angle relative to vertical
  float alpha2 = (UPPER_LEG_LENGTH * UPPER_LEG_LENGTH + xzPlanarReach * xzPlanarReach - LOWER_LEG_LENGTH * LOWER_LEG_LENGTH) /
                     (2 * UPPER_LEG_LENGTH * xzPlanarReach);
  alpha2 = acos(constrain(alpha2, -1.0f, 1.0f)); // Clamp for safety
  float theta2 = alpha2 -  xzPlaneAngle; // Hip angle = foot angle minus geometric offset
  
  // ===== CONVERT TO DEGREES AND RETURN =====
  
  float shoulderAngleDeg = theta1 * 180.0f / PI;
  float hipAngleDeg = theta2 * 180.0f / PI;
  float kneeAngleDeg = theta3 * 180.0f / PI;

  result.shoulder = shoulderAngleDeg;
  result.hip = hipAngleDeg;
  result.knee = kneeAngleDeg;
  result.reachable = true;

  DEBUG_PRINT("IK - Shoulder: ");
  DEBUG_PRINT(shoulderAngleDeg);
  DEBUG_PRINT("° | Hip: ");
  DEBUG_PRINT(hipAngleDeg);
  DEBUG_PRINT("° | Knee: ");
  DEBUG_PRINTLN(kneeAngleDeg);
  DEBUG_PRINT("° (yzReach=");
  DEBUG_PRINT(yzPlanarReach);
  DEBUG_PRINT(" xzReach=");
  DEBUG_PRINT(xzPlanarReach);
  DEBUG_PRINTLN(")");

  return result;
}
