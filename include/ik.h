#pragma once

#include "robot_types.h"

JointAngles computeIK(float targetX, float targetZ, float gaitPhase = 0.0f);
