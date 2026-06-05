#pragma once

#include "robot_types.h"

void cosGait(float phase, LegConfig &leg, int legIndex);
void squareGait(float phase, LegConfig &leg, int legIndex);
void lowerDiagonalLeg(int legIndex);
void setLegZ(int legIndex, float z);
void lowerBodySide(int swingLeg, float amount);
void moveFoot(int legIndex, float x, float y, float z);
void initializeFootPositions();
void shiftBody(int legIndex, float target);
void unshiftBody(int legIndex, float target);