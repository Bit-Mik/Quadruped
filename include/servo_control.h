#pragma once

#include "robot_types.h"

void applyServos(const JointAngles &angles, LegConfig &leg, int legIndex = -1);
