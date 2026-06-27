#pragma once

extern volatile float targetForward;
extern volatile float targetTurn;
enum RobotMode
{
    MODE_STAND,
    MODE_GAIT,
    MODE_MANUAL
};
struct LegConfig {
  int shoulderCh;
  int hipCh;
  int kneeCh;
  float sOffset; //shoulder offset relative to hip in IK coordinate frame (cm)
  float hOffset; //hip offset relative to shoulder in IK coordinate frame (cm)
  float kOffset; //knee offset relative to hip in IK coordinate frame (cm)
  bool isLeftSide;
};

struct JointAngles {
  float shoulder;
  float hip;
  float knee;
  bool reachable;
};

struct SaturationStats {
  int shoulderSaturations[4];
  int hipSaturations[4];
  int kneeSaturations[4];
  unsigned int totalSaturationEvents;
};

struct FootPos
{
    float x;
    float y;
    float z;
};

struct RuntimeConfig
{
    float kp;
    float ki;
    float kd;

    float bodyHeight;

    float stepLength;
    float stepHeight;

    float forwardSpeed;
    float turnSpeed;
};


