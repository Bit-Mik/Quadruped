#pragma once

#include <Arduino.h>

// Manual servo control via serial commands
// Commands:
//   S0=90      : Set servo 0 to 90 degrees
//   S0?        : Get current angle of servo 0
//   ALL=90     : Set ALL servos to 90 degrees
//   HOME       : Set all servos to neutral (90 degrees)
//   FR=10,-19  : Move FR leg to X=10cm, Z=-19cm
//   FL=10,-19  : Move FL leg to X=10cm, Z=-19cm
//   BR=10,-19  : Move BR leg to X=10cm, Z=-19cm
//   BL=10,-19  : Move BL leg to X=10cm, Z=-19cm
//   G          : Start gait
//   STOP       : Stop gait
//   HELP       : Show command list

void printManualControlHelp();
void handleSerialCommand(String command);
void setServoManual(int servoIndex, float angle);
float getServoAngle(int servoIndex);
void setLegPosition(int legIndex, float targetX,float targetY,float targetZ);
