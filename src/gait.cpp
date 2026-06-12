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
float bodyShiftX = 0;
float bodyShiftY = 0;
const int gaitOrder[4] =
{
    LEG_FR,
    LEG_BR,
    LEG_FL,
    LEG_BL
};
float xBias;

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
    if (legIndex == LEG_FR || legIndex == LEG_FL)
        xBias = FRONT_X_BIAS;
    else 
        xBias = REAR_X_BIAS;
    footPos[legIndex].x =
    x + xBias + gaitState.bodyShiftX;

    footPos[legIndex].y =
    y + gaitState.bodyShiftY;
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
    for(int step = 0; step < 4; step++)
    {
        int leg = gaitOrder[step];

        gaitState.phase[leg] =
            fmod(phaseTime + step * 0.25f, 1.0f);
    }

    gaitState.swingLegCount = 0;
    gaitState.leftSwing = false;
    gaitState.rightSwing = false;

    static int lastSwingLeg = -1;

    for(int i = 0; i < 4; i++)
    {
        gaitState.swing[i] =
            gaitState.phase[i] < SWING_PORTION;

        if(gaitState.swing[i])
        {
            if(i != lastSwingLeg)
            {
                lastSwingLeg = i;

                Serial.print("NEW SWING = ");

                switch(i)
                {
                    case LEG_FR: Serial.println("FR"); break;
                    case LEG_FL: Serial.println("FL"); break;
                    case LEG_BR: Serial.println("BR"); break;
                    case LEG_BL: Serial.println("BL"); break;
                }
            }

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
    float targetShiftY = 0.0f;
    float targetShiftX = 0.0f;

    int swingLeg = -1;

    for(int i = 0; i < 4; i++)
    {
        if(gaitState.swing[i])
        {
            swingLeg = i;
            break;
        }
    }

    switch(swingLeg)
    {
        case LEG_FR:
            targetShiftY = +BODY_SHIFT_Y;   // shift left
            targetShiftX = +BODY_SHIFT_X;             // shift rear
            break;

        case LEG_FL:
            targetShiftY = -BODY_SHIFT_Y;   // shift right
            targetShiftX = +BODY_SHIFT_X;             // shift rear
            break;

        case LEG_BR:
            targetShiftY = +BODY_SHIFT_Y;   // shift left
            targetShiftX = +BODY_SHIFT_X;             // shift forward
            break;

        case LEG_BL:
            targetShiftY = -BODY_SHIFT_Y;   // shift right
            targetShiftX = +BODY_SHIFT_X;             // shift forward
            break;
    }

    gaitState.bodyShiftY +=
        SHIFT_SMOOTHING *
        (targetShiftY - gaitState.bodyShiftY);

    gaitState.bodyShiftX +=
        SHIFT_SMOOTHING *
        (targetShiftX - gaitState.bodyShiftX);
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