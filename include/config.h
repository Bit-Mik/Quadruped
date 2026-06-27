#pragma once

#include <Arduino.h>
//========================================Hardware & Control Parameters========================================
constexpr int FREQUENCY = 50; //// PWM update frequency for servos (50Hz = standard for analog servos)
constexpr float PERIOD_MS = 1000.0f / FREQUENCY; // Period of one PWM cycle in milliseconds, Used to convert angle → PCA9685 ticks
constexpr float CONTROL_DT = 0.01f; // Control loop timestep (seconds). 10 ms control update rate 
constexpr float Z_GROUND = -16.0f; // Target Y position of foot on the ground (cm) - within max reach (~22.5cm)
constexpr float X_OFFSET = -1.5f; //(0.0f) Forward/backward offset of foot relative to hip in IK coordinate frame (cm)
constexpr float Y_OFFSET = 0.0f; // Lateral offset of foot relative to shoulder in IK coordinate frame (cm)
// Link lengths (cm or same unit as targetX, targetY)
constexpr float SHOULDER_LENGTH = 3.5f;   // Offset from hip pivot to shoulder servo pivot (cm)(Rz)
constexpr float SHOULDER_WIDTH = 4.1f; // Offset from shoulder pivot to upper leg pivot (cm)(Ry)
constexpr float UPPER_LEG_LENGTH = 10.0f; // Length of upper leg (hip → knee) in cm
constexpr float LOWER_LEG_LENGTH = 10.0f; // Length of lower leg (knee → foot) in cm
constexpr float FRONT_X_BIAS = 0.0f; //5
constexpr float REAR_X_BIAS = 0.0f; //-1

//========================================PID parameters========================================
extern float ROLL_DEADBAND ;  // degrees
extern float PITCH_DEADBAND;  // degrees

extern float KP_ROLL;
extern float KI_ROLL;
extern float KD_ROLL;

extern float KP_PITCH;
extern float KI_PITCH;
extern float KD_PITCH;

extern float MAX_ROLL_CORR; // cm
extern float MAX_PITCH_CORR; // cm

//==================================Gait parameters========================================
// Gait parameters
extern float STEP_LENGTH;
extern float STEP_HEIGHT;
extern float ROLL_COMP;  
extern float BODY_SHIFT_Y;
extern float BODY_SHIFT_X;
extern float BODY_X_TRIM;
extern float BODY_SHIFT_GAIN;
extern float SHIFT_SMOOTHING;
extern float GAIT_CYCLE_DURATION;  // seconds per full gait cycle
// Gait timing
constexpr float SWING_PORTION = 0.20f;
constexpr float STANCE_PORTION = 0.80f;
constexpr float GAIT_START_PHASE = SWING_PORTION;

// Swing subdivision
constexpr float LIFT_END  = 0.20f;
constexpr float SWING_END = 0.80f;
constexpr float LOWER_END = 1.00f;


//=====================================other parameters=====================================
// Leg indices for clarity (matches legs[4] array order in globals.cpp)
constexpr int LEG_FR = 0;  // Front-Right
constexpr int LEG_FL = 1;  // Front-Left
constexpr int LEG_BR = 2;  // Back-Right
constexpr int LEG_BL = 3;  // Back-Left

// Debug & Safety
extern bool DEBUG_MODE;
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr int MIN_SERVO_ANGLE = 0;
constexpr int MAX_SERVO_ANGLE = 180;
constexpr float SINGULARITY_MARGIN = 1.0f;  // Prevent full leg extension

#define DEBUG_PRINT(x) if (DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) Serial.println(x)
