#pragma once

#include "robot_types.h"

struct GaitConfig
{
    float swingPortion;

    float phaseFR;
    float phaseFL;
    float phaseBR;
    float phaseBL;

    float stepLength;
    float stepHeight;
};

struct GaitState
{
    float phase[4];
    bool swing[4];

    bool leftSwing;
    bool rightSwing;

    int swingLegCount;

    float bodyShiftY;
};

extern GaitConfig crawlGait;
extern GaitState gaitState;
// extern GaitConfig trotGait;

void cosGait(float phase, LegConfig &leg, int legIndex);
void squareGait(float phase, LegConfig &leg, int legIndex);
void lowerDiagonalLeg(int legIndex);
void setLegZ(int legIndex, float z);
void lowerBodySide(int swingLeg, float amount);
void moveFoot(int legIndex, float x, float y, float z);
void initializeFootPositions();
void shiftBody(int legIndex, float target);
void unshiftBody(int legIndex, float target);
void updateBodyCompensation();
void stabilizeBody();