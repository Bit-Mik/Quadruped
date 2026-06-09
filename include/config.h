#pragma once

#include <Arduino.h>
//========================================Hardware & Control Parameters========================================
constexpr int FREQUENCY = 50; //// PWM update frequency for servos (50Hz = standard for analog servos)
constexpr float PERIOD_MS = 1000.0f / FREQUENCY; // Period of one PWM cycle in milliseconds, Used to convert angle → PCA9685 ticks
constexpr float CONTROL_DT = 0.01f; // Control loop timestep (seconds). 10 ms control update rate 
constexpr float Z_GROUND = -15.5f; // Target Y position of foot on the ground (cm) - within max reach (~22.5cm)
constexpr float X_OFFSET = -1.5f; //(0.0f) Forward/backward offset of foot relative to hip in IK coordinate frame (cm)
constexpr float Y_OFFSET = 0.5f; // Lateral offset of foot relative to shoulder in IK coordinate frame (cm)
// Link lengths (cm or same unit as targetX, targetY)
constexpr float SHOULDER_LENGTH = 3.5f;   // Offset from hip pivot to shoulder servo pivot (cm)(Rz)
constexpr float SHOULDER_WIDTH = 4.1f; // Offset from shoulder pivot to upper leg pivot (cm)(Ry)
constexpr float UPPER_LEG_LENGTH = 10.0f; // Length of upper leg (hip → knee) in cm
constexpr float LOWER_LEG_LENGTH = 10.0f; // Length of lower leg (knee → foot) in cm

//========================================PID parameters========================================
constexpr float ROLL_DEADBAND  = 0.5f;  // degrees
constexpr float PITCH_DEADBAND = 0.5f;  // degrees

constexpr float KP_ROLL = 0.2f;
constexpr float KI_ROLL = 0.0f;
constexpr float KD_ROLL = 0.05f;

constexpr float KP_PITCH = 0.2f;
constexpr float KI_PITCH = 0.0f;
constexpr float KD_PITCH = 0.05f;

constexpr float MAX_ROLL_CORR  = 0.5f; // cm
constexpr float MAX_PITCH_CORR = 0.5f; // cm

//=========================================Gait parameters========================================
// Gait parameters
constexpr float STEP_LENGTH = 5.0f;
constexpr float STEP_HEIGHT = 0.8f;
constexpr float ROLL_COMP=0.2f;  
constexpr float BODY_SHIFT_GAIN = 0.5f;
constexpr float SHIFT_SMOOTHING = 0.05f;
const float GAIT_CYCLE_DURATION=1.0f;  // seconds per full gait cycle
// Gait timing
constexpr float SWING_PORTION = 0.25f;
constexpr float STANCE_PORTION = 0.75f;

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
