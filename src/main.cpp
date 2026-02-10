#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Wire.h> 
Adafruit_PWMServoDriver pwm;

//======================CONFIGURATION======================
const int FREQUENCY = 50;
const float PERIOD_MS = 1000.0f / FREQUENCY;
const float CONTROL_DT = 0.01f;
const float Y_GROUND = -20.0f;
const float X_OFFSET = -0.0f;
const float HIP_FRAME_ROTATION = -90.0f;

// ---- Link lengths (cm or same unit as targetX, targetY) ----
const float UPPER_LEG_LENGTH = 12.5f;  // upper leg
const float LOWER_LEG_LENGTH = 16.3f;  // lower leg

// ---- Gait parameters ----
const float STEP_LENGTH = 6.0f;        // forward distance per step
const float STEP_HEIGHT = 6.0f;        // lift height during swing phase
float GAIT_CYCLE_DURATION = 3.0f;   // seconds per full gait cycle (adjust for speed)

// ---- Leg indices for clarity ----
const int LEG_BL = 0;  // Back Left
const int LEG_FL = 1;  // Front Left
const int LEG_FR = 3;  // Front Right
const int LEG_BR = 2;  // Back Right

// ---- Debug & Safety ----
const bool DEBUG_MODE = true;
const uint8_t PCA9685_ADDR = 0x40;
const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;
const float SINGULARITY_MARGIN = 1.0f;  // Safety margin (cm) to prevent full leg extension

// Debug helper macros
#define DEBUG_PRINT(x) if(DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG_MODE) Serial.println(x)

//Leg Struct
struct LegConfig {
  int hipCh;
  int kneeCh;
  float hipMechOffset;     // horn calibration (physical truth)
  float kneeMechOffset;
  // float hipCtrlOffset;     // posture / gait bias
  // float kneeCtrlOffset;
  bool isLeftSide;         // mirror symmetry
};

struct JointAngles {
  float hip;
  float knee;
  bool reachable;
};

LegConfig legs[4] = {
  // hipCh, kneeCh, hipMech, kneeMech, isLeft

  { 0,  1,  90, 90, true },   // Back Left
  { 4,  5,  90, 90, true },   // Front Left
  { 8,  9,  90, 90, false },   // Back Right
  {12, 13,  90, 90, false }    // Front RightS
};

// ---- Saturation tracking (for detecting offset misconfiguration) ----
struct SaturationStats {
  int hipSaturations[4];
  int kneeSaturations[4];
  unsigned int totalSaturationEvents;
};
SaturationStats satStats = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0};

//CONSTANTS & STATE
float phaseTime = 0;
unsigned long lastTime = 0;
bool isGaitRunning = false;
bool initializationMode = false;  // Disables saturation tracking during startup


// I2C Device Detection
bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int PWM(float angle) {
  // angle = constrain(angle, 0, 180);
  float pulse = 0.5f + 2.0f * angle / 180;
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

  // Reachability check with singularity safety margin
  // Prevents knee from reaching full extension (singularity dangerous for servos)
  if (distance > (UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - SINGULARITY_MARGIN) || distance < fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)) {
    return result; // unreachable or too close to singularity
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

// Actuator-only: apply computed angles to servos (with safety clamps & saturation tracking)
void applyServos(const JointAngles &angles, LegConfig &leg, int legIndex = -1)
{
  if (!angles.reachable) return;

  /* ---------- 1. IK angles (pure math) ---------- */
  float hipIK  = angles.hip;    // degrees, 0 = neutral
  float kneeIK = angles.knee;   // degrees, 0 = straight

  /* ---------- 2. Mirror geometry (NOT IK math) ---------- */
  // Left legs are mirrored mechanically
  if (leg.isLeftSide) {
    hipIK = -hipIK;
  }
  if (!leg.isLeftSide) {
    kneeIK = -kneeIK;
  }
  hipIK += (leg.isLeftSide ? HIP_FRAME_ROTATION : -HIP_FRAME_ROTATION);  // Rotate hip frame to match physical mounting

  /* ---------- 3. Map to servo angles ---------- */
  float hipServo =
      leg.hipMechOffset +     // horn alignment (physical truth)
      hipIK;                  // IK command

  float kneeServo =
      leg.kneeMechOffset +    // horn alignment
      kneeIK;                 // IK command

  /* ---------- 4. Saturation detection (before clamp) ---------- */
  bool hipSat  = (hipServo < MIN_SERVO_ANGLE || hipServo > MAX_SERVO_ANGLE);
  bool kneeSat = (kneeServo < MIN_SERVO_ANGLE || kneeServo > MAX_SERVO_ANGLE);

  if (!initializationMode && (hipSat || kneeSat)) {
    satStats.totalSaturationEvents++;
    if (legIndex >= 0 && legIndex < 4) {
      if (hipSat)  satStats.hipSaturations[legIndex]++;
      if (kneeSat) satStats.kneeSaturations[legIndex]++;
    }
  }

  /* ---------- 5. Clamp for safety ---------- */
  hipServo  = constrain(hipServo,  MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  kneeServo = constrain(kneeServo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  /* ---------- 6. Send PWM ---------- */
  pwm.setPWM(leg.hipCh,  0, PWM(hipServo));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeServo));

  /* ---------- 7. Debug ---------- */
  if (DEBUG_MODE) {
    Serial.print("Leg "); Serial.print(legIndex);
    Serial.print(" | Hip IK: "); Serial.print(angles.hip);
    Serial.print(" → Servo: "); Serial.print(hipServo);
    Serial.print(" | Knee IK: "); Serial.print(angles.knee);
    Serial.print(" → Servo: "); Serial.println(kneeServo);
  }
}


// Stepping Function (legIndex passed explicitly to avoid runtime lookup)
void stepLeg(float phase, float xOffset, float yGround, LegConfig &leg, int legIndex)
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
    applyServos(jointAngles, leg, legIndex);
  } else {
    DEBUG_PRINTLN("Unreachable target");
  }
}

// Emergency stop: halt all servo motion
void emergencyStop() {
  Serial.println("\n!!! EMERGENCY STOP !!!");
  isGaitRunning = false;
  
  // Return all legs to neutral
  JointAngles neutralAngles = computeIK(X_OFFSET, Y_GROUND);
  if (neutralAngles.reachable) {
    for (int i = 0; i < 4; i++) {
      applyServos(neutralAngles, legs[i], i);
      delay(30);
    }
  }
  
  Serial.println("All servos at neutral. Safe to power down.");
}

// Report saturation statistics
void reportSaturationStats() {
  Serial.println("\n=== Saturation Report ===");
  Serial.print("Total events: "); Serial.println(satStats.totalSaturationEvents);
  
  for (int i = 0; i < 4; i++) {
    if (satStats.hipSaturations[i] > 0 || satStats.kneeSaturations[i] > 0) {
      Serial.print("Leg "); Serial.print(i);
      Serial.print(" - Hip: "); Serial.print(satStats.hipSaturations[i]);
      Serial.print(" | Knee: "); Serial.println(satStats.kneeSaturations[i]);
    }
  }
  
  if (satStats.totalSaturationEvents == 0) {
    Serial.println("No saturation. Offsets tuned well.");
  } else {
    Serial.println("WARNING: Review offset values.");
  }
}

// Initialize all servos to neutral/home position
void initializeServos() {
  Serial.println("\nInitializing servos to standing posture...");
  initializationMode = true;  // Disable saturation tracking during init
  
  // Compute IK for standing position
  JointAngles neutralAngles = computeIK(X_OFFSET, Y_GROUND);
  
  if (!neutralAngles.reachable) {
    Serial.println("ERROR: Standing posture unreachable!");
    while (1);
  }
  
  // Log computed angles
  Serial.print("Neutral IK: Hip="); Serial.print(neutralAngles.hip);
  Serial.print("° Knee="); Serial.print(neutralAngles.knee);
  Serial.println("°");
  
  // Apply and log actual servo angles
  Serial.println("Servo initialization:");
  for (int i = 0; i < 4; i++) {
    float hipBeforeClamp = legs[i].hipMechOffset +
                           (legs[i].isLeftSide ? -neutralAngles.hip : neutralAngles.hip);
    float kneeBeforeClamp = legs[i].kneeMechOffset - neutralAngles.knee;
    float hipClamped = constrain(hipBeforeClamp, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    float kneeClamped = constrain(kneeBeforeClamp, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    
    Serial.print("  Leg "); Serial.print(i);
    Serial.print(" Hip="); Serial.print(hipClamped); Serial.print("°");
    if (hipBeforeClamp != hipClamped) Serial.print("[SAT]");
    Serial.print(" Knee="); Serial.print(kneeClamped); Serial.print("°");
    if (kneeBeforeClamp != kneeClamped) Serial.print("[SAT]");
    Serial.println();
    
    applyServos(neutralAngles, legs[i], i);
    delay(50);
  }
  
  Serial.println("Settling...");
  delay(500);
  
  initializationMode = false;  // Resume normal operation
  // Reset saturation stats (they were disabled during init, but clear for cleanliness)
  satStats.totalSaturationEvents = 0;
  for (int i = 0; i < 4; i++) {
    satStats.hipSaturations[i] = 0;
    satStats.kneeSaturations[i] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  
  Serial.println("\n=== Quadruped Startup ===");
  
  // Verify I2C device
  if (!i2cDevicePresent(PCA9685_ADDR)) {
    Serial.println("ERROR: PCA9685 not found at 0x" + String(PCA9685_ADDR, HEX));
    while (1);
  }
  Serial.println("PCA9685 detected.");

  pwm.begin();
  delay(10);
  pwm.setPWMFreq(FREQUENCY);
  
  // Initialize servos
  initializeServos();
  
  lastTime = millis();
  isGaitRunning = 1;
  Serial.println("Ready. Gait starting...");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      emergencyStop();
      reportSaturationStats();
      return;
    }
  }
  
  if (!isGaitRunning) return;
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < CONTROL_DT) return;   // wait until 10 ms elapsed
  lastTime = now;

  phaseTime += dt / GAIT_CYCLE_DURATION;
  phaseTime = fmod(phaseTime, 1.0f);

  stepLeg(fmod(phaseTime + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BL], LEG_BL);
  stepLeg(fmod(phaseTime + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FL], LEG_FL);
  stepLeg(fmod(phaseTime + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_FR], LEG_FR);
  stepLeg(fmod(phaseTime + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[LEG_BR], LEG_BR);
  // pwm.setPWM(0,  0, PWM(90));   // Hip BL
  // pwm.setPWM(1,  0, PWM(90));   // Knee BL
  // pwm.setPWM(4,  0, PWM(90));   // Hip FL
  // pwm.setPWM(5,  0, PWM(90));   // Knee FL
  // pwm.setPWM(8,  0, PWM(90));   // Hip BR
  // pwm.setPWM(9,  0, PWM(90));   // Knee BR
  // pwm.setPWM(12,  0, PWM(90));  // Hip FR
  // pwm.setPWM(13, 0, PWM(90));   // Knee FR
}

// Leg frame: m = horizontal (±), n = vertical (↓ negative)
// All distances in cm; angles in degrees; time in seconds

