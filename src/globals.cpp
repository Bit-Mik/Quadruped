#include "config.h"
#include "globals.h"

Adafruit_PWMServoDriver pwm;

// Channel mapping: FR: 0,1,2 | FL: 4,5,6 | BR: 8,9,10 | BL: 12,13,14
// Calibration offsets: value - 90 (where 90 is ideal mid position)
LegConfig legs[4] = {
    {0, 1, 2, 0,10,5,false},  // Front-Right: Shoulder(90-90=0), Hip(100-90=10), Knee(90-90=5)
    {4, 5, 6, 0,-5,0,true},   // Front-Left: Shoulder(90-90=0), Hip(85-90=-5), Knee(90-90=0)
    {8, 9, 10, 5,-5,0,false}, // Back-Right: Shoulder(95-90=5), Hip(85-90=-5), Knee(90-90=0)
    {12, 13, 14, -6,-5,0,true}};  // Back-Left: Shoulder(84-90=-6), Hip(85-90=-5), Knee(90-90=0)

SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};

float phaseTime = 0.0f;
unsigned long lastTime = 0;
bool DEBUG_MODE = false;
bool isGaitRunning = false;
bool initializationMode = false;
