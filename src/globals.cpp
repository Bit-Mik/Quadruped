#include "config.h"
#include "globals.h"

Adafruit_PWMServoDriver pwm;

float GAIT_CYCLE_DURATION = 3.0f;

// Channel mapping: FR: 0,1,2 | FL: 4,5,6 | BR: 8,9,10 | BL: 12,13,14
// Calibration offsets: value - 90 (where 90 is ideal mid position)
LegConfig legs[4] = {
    {0, 1, 2, 30,35,30,false},  // Front-Right: Shoulder(120-90=30), Hip(125-90=35), Knee(120-90=30)
    {4, 5, 6, 25,25,30,true},   // Front-Left: Shoulder(115-90=25), Hip(115-90=25), Knee(120-90=30)
    {8, 9, 10, 30,25,25,false}, // Back-Right: Shoulder(120-90=30), Hip(115-90=25), Knee(115-90=25)
    {12, 13, 14, 35,25,25,true}};  // Back-Left: Shoulder(125-90=35), Hip(115-90=25), Knee(115-90=25)

SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};

float phaseTime = 0.0f;
unsigned long lastTime = 0;
bool DEBUG_MODE = false;
bool isGaitRunning = false;
bool initializationMode = false;
