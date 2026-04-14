#pragma once

#include <Arduino.h>

// Manual servo control via serial commands
// Commands:
//   S0=90      : Set servo 0 to 90 degrees
//   S0?        : Get current angle of servo 0
//   ALL=90     : Set ALL servos to 90 degrees
//   HOME       : Set all servos to neutral (90 degrees)
//   G          : Start gait
//   STOP       : Stop gait
//   HELP       : Show command list

void printManualControlHelp();
void handleSerialCommand(String command);
void setServoManual(int servoIndex, float angle);
float getServoAngle(int servoIndex);
