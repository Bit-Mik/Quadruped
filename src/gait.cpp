#include <math.h>

#include "config.h"
#include "gait.h"
#include "ik.h"
#include "servo_control.h"
#include "servo_manual.h"
#include <globals.h>
#include "imu.h"
#include "stabilization.h"

bool reverseGait = false;
FootPos footPos[4];
GaitState gaitState;


//=====================Body Control Functions======================
void initializeFootPositions()
{
    gaitState.bodyShiftY = 0;
    for(int i = 0; i < 4; i++)
    {
        footPos[i].x = X_OFFSET;
        footPos[i].y = Y_OFFSET;
        footPos[i].z = Z_GROUND;
    }
}

bool isSwingPhase(float phase)
{
    return phase < SWING_PORTION;
}

void moveFoot(int legIndex, float x, float y, float z)
{
    float zComp = 0;

bool isSwingLeg = gaitState.swing[legIndex];

if(!isSwingLeg)
{
    if(legIndex == LEG_FL || legIndex == LEG_BL)
        zComp += getRollCompLeft();
    else
        zComp += getRollCompRight();

    if(legIndex == LEG_FL || legIndex == LEG_FR)
        zComp += getPitchCompFront();
    else
        zComp += getPitchCompRear();
}
    footPos[legIndex].x = x;
    footPos[legIndex].y =
    y + gaitState.bodyShiftY;;
    footPos[legIndex].z = z + zComp;

    JointAngles ja = computeIK(
        footPos[legIndex].x,
        footPos[legIndex].y,
        footPos[legIndex].z
    );

    applyServos(ja, legs[legIndex], legIndex);
}

void updateGaitState(float phaseTime)
{
    gaitState.phase[LEG_BL] =
        fmod(phaseTime + 0.00f, 1.0f);

    gaitState.phase[LEG_FR] =
        fmod(phaseTime + 0.25f, 1.0f);

    gaitState.phase[LEG_FL] =
        fmod(phaseTime + 0.50f, 1.0f);

    gaitState.phase[LEG_BR] =
        fmod(phaseTime + 0.75f, 1.0f);

    gaitState.swingLegCount = 0;

    gaitState.leftSwing = false;
    gaitState.rightSwing = false;

    for(int i = 0; i < 4; i++)
    {
        gaitState.swing[i] =
            gaitState.phase[i] < SWING_PORTION;

        if(gaitState.swing[i])
        {
            gaitState.swingLegCount++;

            if(i == LEG_FL || i == LEG_BL)
                gaitState.leftSwing = true;

            if(i == LEG_FR || i == LEG_BR)
                gaitState.rightSwing = true;
        }
    }
}

void updateSupportShift()
{
    float targetShift = 0;

    if(gaitState.leftSwing)
    targetShift += BODY_SHIFT_GAIN;

    if(gaitState.rightSwing)
    targetShift -= BODY_SHIFT_GAIN;

    gaitState.bodyShiftY +=
        SHIFT_SMOOTHING *
        (targetShift - gaitState.bodyShiftY);
}


//=====================GAIT FUNCTIONS======================

void cosGait(float phase, LegConfig &leg, int legIndex)
{
    float X, Y, Z;

    // Keep phase in [0,1)
    phase = fmod(phase, 1.0f);

    if (phase < 0.5f)
    {
        // ======================
        // Swing Phase
        // ======================
        float t = phase / 0.5f;   // 0 → 1

        if (!reverseGait)
            X = X_OFFSET -STEP_LENGTH/2 + STEP_LENGTH * t;
        else
            X = X_OFFSET + STEP_LENGTH/2 - STEP_LENGTH * t;

        Z = Z_GROUND + STEP_HEIGHT * cos(X/STEP_LENGTH * PI);
    }
    else
    {
        // ======================
        // Stance Phase
        // ======================
        float t = (phase - 0.5f) / 0.5f;   // 0 → 1

        if (!reverseGait)
            X = X_OFFSET + STEP_LENGTH/2 - STEP_LENGTH * t;
        else
            X = X_OFFSET - STEP_LENGTH/2 + STEP_LENGTH * t;

        Z = Z_GROUND;
    }

    Y = Y_OFFSET;

    moveFoot(legIndex, X, Y, Z);
}

void squareGait(float phase, LegConfig &leg, int legIndex)
{
    float X, Y, Z;

    const float L = STEP_LENGTH;
    const float H = STEP_HEIGHT;

    phase = fmod(phase, 1.0f);

    // ==========================
    // Swing phase
    // ==========================
    if (phase < SWING_PORTION)
    {
        float t = phase / SWING_PORTION;

        // Lift
        if (t < LIFT_END)
        {
            float s = t / LIFT_END;

            X = X_OFFSET - L/2;
            Z = Z_GROUND + H * s;
        }

        // Forward
        else if (t < SWING_END)
        {
            float s = (t - LIFT_END) /
                      (SWING_END - LIFT_END);

            X = X_OFFSET - L/2 + L * s;
            Z = Z_GROUND + H;
        }

        // Lower
        else
        {
            float s = (t - SWING_END) /
                      (LOWER_END - SWING_END);

            X = X_OFFSET + L/2;
            Z = Z_GROUND + H * (1.0f - s);
        }
    }

    // ==========================
    // Stance phase
    // ==========================
    else
    {
        float t = (phase - SWING_PORTION) /
                  STANCE_PORTION;

        X = X_OFFSET + L/2 - L * t;
        Z = Z_GROUND;
    }

    Y = Y_OFFSET;

    moveFoot(legIndex, X, Y, Z);
}