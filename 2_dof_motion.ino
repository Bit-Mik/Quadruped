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
  int hipDir;    // +1 or -1
  int kneeDir;   // +1 or -1
};

LegConfig legs[4] = {
  // hipCh, kneeCh, hipOffset, kneeOffset, hipDir, kneeDir

  { 0,  1, 180, 90, -1, -1 },   // Front Left
  { 4,  5,   0, 90, +1, +1 },   // Front Right
  { 8,  9,   0, 90, +1, +1 },   // Back Right
  {12, 13, 180, 90, -1, -1 }    // Back Left
};


//CONSTANTS 
// float m =0;    // x coordinate
// float n = -20;  // y coordinate (downward negative)
float t = 0;
unsigned long last = 0;
int freq = 50;
const float PERIOD = 1000.0f / freq;
const float CONTROL_DT = 0.01f;
const float Y_GROUND = -25;
const float X_OFFSET = 0;
// ---- Link lengths (cm or same unit as m,n) ----
const float x = 12.5f;  // upper leg
const float y = 16.3f;  // lower leg

// I2C Device Detection
bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int PWM(float angle) {
  float pulse = 0.5f + 2 * angle / 180;
  int ticks = round(pulse * 4096 / PERIOD);
  // Clamp ticks to valid PCA9685 range
  ticks = constrain(ticks, 0, 4095);
  return ticks;
}

//IK Calculations
// IK result (compute-only)
struct JointAngles {
  float hip;
  float knee;
  bool reachable;
};

JointAngles computeIK(float m, float n, LegConfig &leg) {
  JointAngles out;
  out.reachable = false;

  float r = sqrt(m * m + n * n);

  // Reachability check
  if (r > (x + y) || r < fabsf(x - y)) {
    return out; // unreachable
  }

  // ---- Knee angle (b) ----
  float cosB = (m * m + n * n - x * x - y * y) / (2 * x * y);
  cosB = constrain(cosB, -1.0f, 1.0f);
  float b = acos(cosB);

  // ---- Hip angle (a) ----
  float a = atan2(n, m) - atan2(y * sin(b), x + y * cos(b));

  // Convert to degrees explicitly
  float aDeg = a * 180.0f / PI;
  float bDeg = b * 180.0f / PI;

  // Compute raw servo angles (without final clamping)
  out.hip = leg.hipOffset + leg.hipDir * aDeg;
  out.knee = leg.kneeOffset + leg.kneeDir * bDeg;
  out.reachable = true;
  return out;
}

// Actuator-only: apply computed angles to servos (with safety clamps)
void applyServos(const JointAngles &angles, LegConfig &leg) {
  if (!angles.reachable) return;

  float hipAngle = constrain(angles.hip, 0, 180);
  float kneeAngle = constrain(angles.knee, 0, 180);

  pwm.setPWM(leg.hipCh,  0, PWM(hipAngle));
  pwm.setPWM(leg.kneeCh, 0, PWM(kneeAngle));
}

//Stepping Function
void stepLeg(float phase, float xOffset, float yGround, LegConfig &leg)
{
  const float stepLength = 6;   // forward distance
  const float stepHeight = 9;   // lift height

  float m, n;

  if (phase < 0.5f) {
    // swing phase (leg in air)
    float phaseT = phase/0.5f;
    m = xOffset + stepLength * (phaseT - 0.5f);
    n = yGround + stepHeight * sin(PI * phaseT);
  }
  else {
    // stance phase (leg on ground pushing back)
    float phaseT = (phase - 0.5f)/ 0.5f;
    m = xOffset + stepLength * (0.5f - phaseT);
    n = yGround;
  }

  // Compute IK then apply to servos (separated for testability)
  JointAngles ja = computeIK(m, n, leg);
  if (ja.reachable) {
    applyServos(ja, leg);
  } else {
    // Optionally: handle unreachable target (log or move to safe pose)
    // Serial.println("Unreachable target for leg");
  }
}



// Servo hipServo;
// Servo kneeServo;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!i2cDevicePresent(0x40)) {
  Serial.println("ERROR: PCA9685 not found");
  while (1); // stop
  }

  pwm.begin();

  delay(10);
  pwm.setPWMFreq(freq);
  last = millis();

  // moveToPoint(m, n, legs[0]);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last) / 1000.0f;

    if (dt < CONTROL_DT) {
    return;   // wait until 10 ms elapsed
  }
  // last += (unsigned long)(CONTROL_DT * 1000);
  last = now;

  t += dt;
  t= fmod(t,1.0f);

  stepLeg(fmod(t + 0.00f, 1.0f), X_OFFSET, Y_GROUND, legs[0]); // FL
  stepLeg(fmod(t + 0.25f, 1.0f), X_OFFSET, Y_GROUND, legs[1]); // FR
  stepLeg(fmod(t + 0.50f, 1.0f), X_OFFSET, Y_GROUND, legs[2]); // BR
  stepLeg(fmod(t + 0.75f, 1.0f), X_OFFSET, Y_GROUND, legs[3]); // BL  

}

