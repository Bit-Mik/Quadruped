#include <math.h>

#include "config.h"
#include "gait.h"
#include "ik.h"
#include "servo_control.h"
#include "servo_manual.h"
#include <globals.h>

bool reverseGait = false;
int steps = STEP_LENGTH / STEPDX;
FootPos footPos[4];
void shiftBody(int swingLeg, float target)
{
    for(float a = 0; a <= target; a += 0.1f)
    {
        lowerBodySide(swingLeg, a);
        delay(10);
    }
}

void unshiftBody(int swingLeg, float target)
{
    for(float a = target; a >= 0; a -= 0.1f)
    {
        lowerBodySide(swingLeg, a);
        delay(10);
    }
}

void initializeFootPositions()
{
    for(int i = 0; i < 4; i++)
    {
        footPos[i].x = X_OFFSET;
        footPos[i].y = Y_OFFSET;
        footPos[i].z = Z_GROUND;
    }
}

void moveFoot(int legIndex, float x, float y, float z)
{
    footPos[legIndex].x = x;
    footPos[legIndex].y = y;
    footPos[legIndex].z = z;

    JointAngles ja = computeIK(x, y, z);
    applyServos(ja, legs[legIndex], legIndex);
}

void setLegZ(int legIndex, float z)
{
    

    moveFoot(legIndex,
            footPos[legIndex].x,
            footPos[legIndex].y,
            z);
}


void lowerDiagonalLeg(int legIndex)
{
    int diag = 3 - legIndex;

    setLegZ(diag, Z_GROUND - ROLL_COMP);
}

void lowerBodySide(int swingLeg, float amount)
{
    bool swingIsLeft =
        (swingLeg == LEG_FL || swingLeg == LEG_BL);

    if(swingIsLeft)
    {
        setLegZ(LEG_FR, Z_GROUND + amount);
        setLegZ(LEG_BR, Z_GROUND + amount);
    }
    else
    {
        setLegZ(LEG_FL, Z_GROUND + amount);
        setLegZ(LEG_BL, Z_GROUND + amount);
    }
}

void cosGait(float phase,LegConfig &leg,int legIndex) {
  float X=X_OFFSET,Y=Y_OFFSET,Z=Z_GROUND;
  X = -STEP_LENGTH/2;
  int n = 0;
  while(X < STEP_LENGTH/2){
    if (!reverseGait)
    X = -STEP_LENGTH/2 + STEPDX*n;
    else
    X = STEP_LENGTH/2 - STEPDX*n;
    Y=Y_OFFSET;
    Z = Z_GROUND + STEP_HEIGHT * cos(X/STEP_LENGTH * PI);
    moveFoot(legIndex, X, Y, Z);
    delay(50); 
    n++;
  }

  //Ground trajectory
  X=STEP_LENGTH/2;
  n = 0;
  while(X > -STEP_LENGTH/2){
    if (!reverseGait)
    X = STEP_LENGTH/2 - STEPDX*n;
    else
    X = -STEP_LENGTH/2 + STEPDX*n;
    Y=Y_OFFSET;
    Z = Z_GROUND;
    moveFoot(legIndex, X, Y, Z);
    delay(50); // Faster at start/end, slower at mid-step
    n++;
  }

}

void squareGait(float phase, LegConfig &leg, int legIndex)
{
  // shiftBody(legIndex, ROLL_COMP);
  // delay(30);
    float X=X_OFFSET, Y=Y_OFFSET, Z;
    int n;

    // Segment lengths
    float L = STEP_LENGTH;
    float H = STEP_HEIGHT;
    float liftSteps = H / STEPDX;

    // ======================
    // 1. Lift vertically
    // ======================
    n = 0;
    Z = Z_GROUND;

    while (Z < Z_GROUND + H)
    {
        Z = Z_GROUND + STEPDX * n;
        float roll = ROLL_COMP * (n / liftSteps);
        lowerBodySide(legIndex, roll);

        if (Z > Z_GROUND + H)
            Z = Z_GROUND + H;

        X = reverseGait ? X_OFFSET + L/2 : X_OFFSET - L/2;
        Y = Y_OFFSET;

        moveFoot(legIndex, X, Y, Z);

        delay(20);
        n++;
    }

    // ======================
    // 2. Move forward in air
    // ======================
    n = 0;
    X = reverseGait ? X_OFFSET + L/2 : X_OFFSET - L/2;

    while ((!reverseGait && X < X_OFFSET + L/2) ||
           ( reverseGait && X > X_OFFSET - L/2)) 
    {
        if (!reverseGait)
            X = X_OFFSET - L/2 + STEPDX * n;
        else
            X = X_OFFSET + L/2 - STEPDX * n;

        Y = Y_OFFSET;
        Z = Z_GROUND + H;

        moveFoot(legIndex, X, Y, Z);

        delay(20);
        n++;
    }

    // ======================
    // 3. Lower vertically
    // ======================
    n = 0;
    Z = Z_GROUND + H;

    while (Z > Z_GROUND)
    {
        Z = Z_GROUND + H - STEPDX * n;
        float roll = ROLL_COMP *
            (1.0f - n / liftSteps);
          lowerBodySide(legIndex, roll);

        if (Z < Z_GROUND)
            Z = Z_GROUND;

        X = reverseGait ? X_OFFSET - L/2 : X_OFFSET + L/2;
        Y = Y_OFFSET;

        moveFoot(legIndex, X, Y, Z);

        delay(20);
        n++;
    }

    // ======================
    // 4. Ground return
    // ======================
    n = 0;
    X = reverseGait ? X_OFFSET - L/2 : X_OFFSET + L/2;

    while ((!reverseGait && X > X_OFFSET - L/2) ||
           ( reverseGait && X < X_OFFSET + L/2))
    {
        if (!reverseGait)
            X = X_OFFSET + L/2 - STEPDX * n;
        else
            X = X_OFFSET - L/2 + STEPDX * n;

        Y = Y_OFFSET;
        Z = Z_GROUND;

        moveFoot(legIndex, X, Y, Z);

        delay(20);
        n++;
    }
    // unshiftBody(legIndex, ROLL_COMP);
}