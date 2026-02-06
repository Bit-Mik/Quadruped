#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Wire.h> 
Adafruit_PWMServoDriver pwm;

//Leg Struct
struct LegConfig {
  int hipCh;
  int kneeCh;

  float hipMechOffset;     // horn calibration (physical truth)
  float kneeMechOffset;

  float hipCtrlOffset;     // posture / gait bias
  float kneeCtrlOffset;

  bool isLeftSide;         // mirror symmetry
};


struct JointAngles {
  float hip;
  float knee;
  bool reachable;
};

LegConfig legs[4] = {
  // hipCh, kneeCh, hipMech, kneeMech, hipCtrl, kneeCtrl, isLeft

  { 0,  1,  90, 90,  0,  70, true  },   // Back Left
  { 4,  5,  90, 90,  0,  70, true  },   // Front Left
  { 8,  9,  90, 90,  0,  70, false },   // Front Right
  {12, 13,  90, 90,  0,  70, false }    // Back Right
};



//CONSTANTS 
float phaseTime = 0;
unsigned long lastTime = 0;
const int FREQUENCY = 50;
const float PERIOD_MS = 1000.0f / FREQUENCY;
const float CONTROL_DT = 0.01f;
const float Y_GROUND = -25;
const float X_OFFSET = 0;

// ---- Link lengths (cm or same unit as targetX, targetY) ----
const float UPPER_LEG_LENGTH = 12.5f;  // upper leg
const float LOWER_LEG_LENGTH = 16.3f;  // lower leg

// ---- Gait parameters ----
const float STEP_LENGTH = 6.0f;        // forward distance per step
const float STEP_HEIGHT = 9.0f;        // lift height during swing phase

// ---- Leg indices for clarity ----
const int LEG_BL = 0;  // Back Left
const int LEG_FL = 1;  // Front Left
const int LEG_FR = 2;  // Front Right
const int LEG_BR = 3;  // Back Right

// ---- Debug & Safety ----
const bool DEBUG_MODE = true;
const uint8_t PCA9685_ADDR = 0x40;
const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;
const float NEUTRAL_HIP_IK = 0.0f;      // neutral IK position
const float NEUTRAL_KNEE_IK = 90.0f;    // neutral IK position

// Debug helper macros
#define DEBUG_PRINT(x) if(DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG_MODE) Serial.println(x)

// I2C Device Detection
bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int PWM(float angle) {
  // angle = constrain(angle, 0, 180);
  float pulse = 0.5f + 2 * angle / 180;
  int ticks = round(pulse * 4096 / PERIOD_MS);
  // Clamp ticks to valid PCA9685 range
  ticks = constrain(ticks, 0, 4095);
  return ticks;
}

//IK Calculations

JointAngles computeIK(float targetX, float targetY) {
  JointAngles result;
  result.reachable = false;

  float distance = sqrt(targetX * targetX + targetY * targetY);

  // Reachability check
  if (distance > (UPPER_LEG_LENGTH + LOWER_LEG_LENGTH) || distance < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)) {
    return result; // unreachable
  }

  // ---- Knee angle (b) ----
  float cosBeta = (targetX * targetX + targetY * targetY - UPPER_LEG_LENGTH * UPPER_LEG_LENGTH - LOWER_LEG_LENGTH * LOWER_LEG_LENGTH) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);
  cosBeta = constrain(cosBeta, -1.0f, 1.0f);
  float kneeAngleRad = acos(cosBeta);

  // ---- Hip angle (a) ----
  float hipAngleRad = atan2(targetY, targetX) - atan2(LOWER_LEG_LENGTH * sin(kneeAngleRad), UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * cos(kneeAngleRad));

  // Convert to degrees explicitly
  float hipAngleDeg = hipAngleRad * 180.0f / PI;
  float kneeAngleDeg = kneeAngleRad * 180.0f / PI;
  result.hip  = hipAngleDeg;
  result.knee = kneeAngleDeg;
  result.reachable = true;
  
  DEBUG_PRINT("IK - Hip deg: ");
  DEBUG_PRINT(hipAngleDeg);
  DEBUG_PRINT(" | Knee deg: ");
  DEBUG_PRINTLN(kneeAngleDeg);
  
  return result;
}

// Actuator-only: apply computed angles to servos (with safety clamps)
void applyServos(const JointAngles &angles, LegConfig &leg) {
  if (!angles.reachable) return;

  float hipIK  = angles.hip;
  float kneeIK = angles.knee;

   // Mirror in IK space
  if (leg.isLeftSide) {
    hipIK = -hipIK;
  }

  float hipServo =
      leg.hipMechOffset +
      leg.hipCtrlOffset +
      hipIK;

  float kneeServo =
      leg.kneeMechOffset +
      leg.kneeCtrlOffset -
      kneeIK;

  hipServo = constrain(hipServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  kneeServo = constrain(kneeServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  pwm.setPWM(leg.hipCh,  0, PWM(hipServo));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeServo));

  DEBUG_PRINT("Hip Servo: "); DEBUG_PRINT(hipServo);
  DEBUG_PRINT(" | Knee Servo: "); DEBUG_PRINTLN(kneeServo);
}

//Stepping Function
void stepLeg(float phase, float xOffset, float yGround, LegConfig &leg)
{
  float targetX, targetY;

  if (phase < 0.5f) {
    // swing phase (leg in air)
    float phaseFraction = phase/0.5f;
    targetX = xOffset + STEP_LENGTH * (phaseFraction - 0.5f);
    targetY = yGround + STEP_HEIGHT * sin(PI * phaseFraction);
  }
  else {
    // stance phase (leg on ground pushing back)
    float phaseFraction = (phase - 0.5f)/ 0.5f;
    targetX = xOffset + STEP_LENGTH * (0.5f - phaseFraction);
    targetY = yGround;
  }

  // Compute IK then apply to servos 
  JointAngles jointAngles = computeIK(targetX, targetY);
  if (jointAngles.reachable) {
    applyServos(jointAngles, leg);
  } else {
    DEBUG_PRINTLN("Unreachable target");
  }
}

// Initialize all servos to neutral/home position
void initializeServos() {
  DEBUG_PRINTLN("Initializing servos to neutral position...");
  
  // Create neutral angles
  JointAngles neutralAngles;
  neutralAngles.hip = NEUTRAL_HIP_IK;
  neutralAngles.knee = NEUTRAL_KNEE_IK;
  neutralAngles.reachable = true;
  
  // Apply neutral position to all legs
  for (int i = 0; i < 4; i++) {
    applyServos(neutralAngles, legs[i]);
    delay(50);  // Small delay between leg updates
  }
  
  DEBUG_PRINTLN("Servos initialized.");
  delay(500);  // Wait for servos to settle
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  
  Serial.println("\n=== Quadruped Startup ===");
  
  // Verify I2C device
  if (!i2cDevicePresent(PCA9685_ADDR)) {
    Serial.println("ERROR: PCA9685 not found at 0x" + String(PCA9685_ADDR, HEX));
    while (1); // halt
  }
  Serial.println("PCA9685 detected.");

  pwm.begin();
  delay(10);
  pwm.setPWMFreq(FREQUENCY);
  
  // Initialize servos to safe position
  initializeServos();
  
  lastTime = millis();
  Serial.println("Quadruped ready. Starting gait.");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < CONTROL_DT) return;   // wait until 10 ms elapsed
  // lastTime += (unsigned long)(CONTROL_DT * 1000);
  lastTime = now;

  phaseTime += dt;
  phaseTime = fmod(phaseTime, 1.0f);

  stepLeg(fmod(phaseTime + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BL]); // Back Left
  stepLeg(fmod(phaseTime + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FL]); // Front Left
  stepLeg(fmod(phaseTime + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FR]); // Front Right
  stepLeg(fmod(phaseTime + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BR]); // Back Right
}

// Leg frame: m = horizontal (±), n = vertical (↓ negative)
// All distances in cm; angles in degrees; time in seconds

