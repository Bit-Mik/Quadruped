#include "config.h"
#include "globals.h"

Adafruit_PWMServoDriver pwm;
volatile RobotMode robotMode = MODE_MANUAL;


// Channel mapping: FR: 0,1,2 | FL: 4,5,6 | BR: 8,9,10 | BL: 12,13,14
// Calibration offsets: value - 90 (where 90 is ideal mid position)
LegConfig legs[4] = {
    {0, 1, 2, 0,10,5,false},  // Front-Right: Shoulder(90-90=0), Hip(100-90=10), Knee(90-90=5)
    {4, 5, 6, 0,-3,0,true},   // Front-Left: Shoulder(90-90=-0), Hip(87-90=-3), Knee(90-90=0)
    {8, 9, 10, 3,-5,0,false}, // Back-Right: Shoulder(93-90=3), Hip(85-90=-5), Knee(90-90=0)
    {12, 13, 14, -5,-5,0,true}};  // Back-Left: Shoulder(85-90=-5), Hip(85-90=-5), Knee(90-90=0)

SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};

//========================================PID parameters========================================
float ROLL_DEADBAND  = 0.5f;  // degrees
float PITCH_DEADBAND = 0.5f;  // degrees

float KP_ROLL = 0.1f;
float KI_ROLL = 0.0f;
float KD_ROLL = 0.00f;

float KP_PITCH = 0.1f;
float KI_PITCH = 0.0f;
float KD_PITCH = 0.00f;

float MAX_ROLL_CORR  = 1.0f; // cm
float MAX_PITCH_CORR = 1.0f; // cm

//=========================================Gait parameters========================================
// Gait parameters
float STEP_LENGTH = 3.0f;
float STEP_HEIGHT = 2.5f;
float ROLL_COMP=0.2f;  
float BODY_SHIFT_X = 0.8f;
float BODY_SHIFT_Y = 0.8f;
float BODY_X_TRIM = -1.0f;
float BODY_SHIFT_GAIN = 0.5f;
float SHIFT_SMOOTHING = 0.05f;
float GAIT_CYCLE_DURATION=5.0f;  // seconds per full gait cycle

volatile float targetForward = 0.0f;
volatile float targetTurn = 0.0f;

volatile float phaseTime = 0.0f;
unsigned long lastTime = 0;
bool DEBUG_MODE = false;
bool initializationMode = false;
