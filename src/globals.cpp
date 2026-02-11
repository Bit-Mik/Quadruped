#include "config.h"
#include "globals.h"

Adafruit_PWMServoDriver pwm;

float GAIT_CYCLE_DURATION = 3.0f;

LegConfig legs[4] = {
    {0, 1, 90, 90, true},
    {4, 5, 90, 90, true},
    {8, 9, 90, 90, false},
    {12, 13, 90, 90, false}};

SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0};

float phaseTime = 0.0f;
unsigned long lastTime = 0;
bool DEBUG_MODE = false;
bool isGaitRunning = false;
bool initializationMode = false;
