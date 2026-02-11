#pragma once

struct LegConfig {
  int hipCh;
  int kneeCh;
  float hipMechOffset;
  float kneeMechOffset;
  bool isLeftSide;
};

struct JointAngles {
  float hip;
  float knee;
  bool reachable;
};

struct SaturationStats {
  int hipSaturations[4];
  int kneeSaturations[4];
  unsigned int totalSaturationEvents;
};
