#pragma once

#include <Arduino.h>

constexpr int FREQUENCY = 50; //// PWM update frequency for servos (50Hz = standard for analog servos)
constexpr float PERIOD_MS = 1000.0f / FREQUENCY; // Period of one PWM cycle in milliseconds, Used to convert angle → PCA9685 ticks
constexpr float CONTROL_DT = 0.01f; // Control loop timestep (seconds). 10 ms control update rate
constexpr float Y_GROUND = -20.0f;
constexpr float X_OFFSET = -0.0f;
constexpr float HIP_FRAME_ROTATION = -90.0f; // Rotates IK coordinate frame to match how hip servo is mounted

// Link lengths (cm or same unit as targetX, targetY)
constexpr float UPPER_LEG_LENGTH = 12.5f; // Length of upper leg (hip → knee) in cm
constexpr float LOWER_LEG_LENGTH = 16.3f; // Length of lower leg (knee → foot) in cm

// Gait parameters
constexpr float STEP_LENGTH = 6.0f;
constexpr float STEP_HEIGHT = 6.0f;
extern float GAIT_CYCLE_DURATION;  // seconds per full gait cycle

// Leg indices for clarity
constexpr int LEG_BL = 0;  // Back Left
constexpr int LEG_FL = 1;  // Front Left
constexpr int LEG_BR = 2;  // Back Right
constexpr int LEG_FR = 3;  // Front Right

// Debug & Safety
extern bool DEBUG_MODE;
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr int MIN_SERVO_ANGLE = 0;
constexpr int MAX_SERVO_ANGLE = 180;
constexpr float SINGULARITY_MARGIN = 1.0f;  // Prevent full leg extension

#define DEBUG_PRINT(x) if (DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) Serial.println(x)
