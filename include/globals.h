#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "robot_types.h"

extern Adafruit_PWMServoDriver pwm;
extern LegConfig legs[4];
extern SaturationStats satStats;
extern RuntimeConfig cfg;

extern float phaseTime;
extern unsigned long lastTime;
extern bool isGaitRunning;
extern bool initializationMode;
