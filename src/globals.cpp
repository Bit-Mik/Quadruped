#include "config.h"
#include "globals.h"

Adafruit_PWMServoDriver pwm;

float GAIT_CYCLE_DURATION = 3.0f;

// Channel mapping: BL: 0,1,2 | BR: 4,5,6 | FL: 8,9,10 | FR: 12,13,14
LegConfig legs[4] = {
    {0, 1, 2, 270, 90, 90, true},   // Back-Left: Shoulder, Hip, Knee (inverted mounting handled in servo_control)
    {8, 9, 10, 90, 90, 90, true},  // Front-Left: Shoulder, Hip, Knee
    {4, 5, 6, 270, 90, 90, false},  // Back-Right: Shoulder, Hip, Knee (inverted mounting handled in servo_control)
    {12, 13, 14, 90, 90, 90, false}}; // Front-Right: Shoulder, Hip, Knee

SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};

float phaseTime = 0.0f;
unsigned long lastTime = 0;
bool DEBUG_MODE = false;
bool isGaitRunning = false;
bool initializationMode = false;
