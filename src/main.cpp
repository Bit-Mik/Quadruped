#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Wire.h> 
Adafruit_PWMServoDriver pwm;

//Leg Struct
struct LegConfig {
  int hipCh;
  int kneeCh;
  float hipOffset;
  float kneeOffset;
  bool isLeftSide;  // true for FL/BL, false for BR/FR - determines mirror symmetry
};

struct JointAngles {
  float hip;
  float knee;
  bool reachable;
};


LegConfig legs[4] = {
  // hipCh, kneeCh, hipOffset, kneeOffset, isLeftSide

  { 0,  1, 180, 180, true },   // Back Left (phase 0.00) - channel 0
  { 4,  5, 180, 180, true },   // Front Left (phase 0.25) - channel 4
  { 8,  9, 180, 180, false },  // Front Right (phase 0.50) - channel 8
  {12, 13, 180, 180, false }   // Back Right (phase 0.75) - channel 12
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

JointAngles computeIK(float targetX, float targetY, LegConfig &leg) {
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

  // Compute servo angles (matched to physical servo orientation)
  result.hip = leg.hipOffset + hipAngleDeg;
  result.knee = leg.kneeOffset - kneeAngleDeg;
  result.reachable = true;
  
  Serial.print("IK - Hip deg: ");
  Serial.print(hipAngleDeg);
  Serial.print(" -> After offset: ");
  Serial.print(result.hip);
  Serial.print(" | Knee deg: ");
  Serial.print(kneeAngleDeg);
  Serial.print(" -> After offset: ");
  Serial.println(result.knee);
  
  return result;
}

// Actuator-only: apply computed angles to servos (with safety clamps)
void applyServos(const JointAngles &angles, LegConfig &leg) {
  if (!angles.reachable) return;

  float hipAngle = constrain(angles.hip, 0, 180);
  float kneeAngle = constrain(angles.knee, 0, 180);

  // Apply mirror symmetry for left-side legs (FL, BL): servo = 180 - angle
  // Right-side legs (BR, FR) use angle directly
  if (leg.isLeftSide) {
    float hipBefore = hipAngle;
    float kneeBefore = kneeAngle;
    hipAngle = 180.0f - hipAngle;
    kneeAngle = 180.0f - kneeAngle;
    Serial.print("Mirror symmetry - Hip: ");
    Serial.print(hipBefore);
    Serial.print(" -> ");
    Serial.print(hipAngle);
    Serial.print(" | Knee: ");
    Serial.print(kneeBefore);
    Serial.print(" -> ");
    Serial.println(kneeAngle);
  }

  pwm.setPWM(leg.hipCh,  0, PWM(hipAngle));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeAngle));
}

//Stepping Function
void stepLeg(float phase, float xOffset, float yGround, LegConfig &leg)
{
  const float STEP_LENGTH = 6;   // forward distance
  const float STEP_HEIGHT = 9;   // lift height

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
  JointAngles jointAngles = computeIK(targetX, targetY, leg);
  if (jointAngles.reachable) {
    applyServos(jointAngles, leg);
  } else {
    Serial.println("Unreachable target for leg");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // if (!i2cDevicePresent(0x40)) {
  // Serial.println("ERROR: PCA9685 not found");
  // while (1); // stop
  // }

  pwm.begin();

  delay(10);
  pwm.setPWMFreq(FREQUENCY);
  lastTime = millis();
  Serial.println("Quadruped initialized. Starting gait.");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < CONTROL_DT) return;   // wait until 10 ms elapsed
  // lastTime += (unsigned long)(CONTROL_DT * 1000);
  lastTime = now;

  phaseTime += dt;
  phaseTime = fmod(phaseTime, 1.0f);

  stepLeg(fmod(phaseTime + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[0]); // FL
  stepLeg(fmod(phaseTime + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[1]); // BR
  stepLeg(fmod(phaseTime + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[2]); // FR
  stepLeg(fmod(phaseTime + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[3]); // BL
}

// Leg frame: m = horizontal (±), n = vertical (↓ negative)
// All distances in cm; angles in degrees; time in seconds

