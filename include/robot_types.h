#pragma once

struct LegConfig {
  int shoulderCh;
  int hipCh;
  int kneeCh;
  float shoulderMechOffset;
  float hipMechOffset;
  float kneeMechOffset;
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
