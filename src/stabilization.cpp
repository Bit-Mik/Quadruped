#include "stabilization.h"
#include "pid.h"
#include "config.h"
#include "imu.h"
#include "globals.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE stabilizationMux = portMUX_INITIALIZER_UNLOCKED;
#define STABILIZATION_LOCK() portENTER_CRITICAL(&stabilizationMux)
#define STABILIZATION_UNLOCK() portEXIT_CRITICAL(&stabilizationMux)
#else
#define STABILIZATION_LOCK()
#define STABILIZATION_UNLOCK()
#endif

static PID rollPID(
    KP_ROLL,
    KI_ROLL,
    KD_ROLL);

static PID pitchPID(
    KP_PITCH,
    KI_PITCH,
    KD_PITCH);

static float rollCompLeft = 0;
static float rollCompRight = 0;

static float pitchCompFront = 0;
static float pitchCompRear = 0;

static float rollSetpoint = 0.0f;
static float pitchSetpoint = 0.0f;

float getRollCompLeft()
{
    return rollCompLeft;
}

float getRollCompRight()
{
    return rollCompRight;
}

float getPitchCompFront()
{
    return pitchCompFront;
}

float getPitchCompRear()
{
    return pitchCompRear;
}

void calibrateBodyPose()
{
    const int samples = 200;

    float rollSum = 0.0f;
    float pitchSum = 0.0f;

    while (fabs(imuGetState().rollRateDps) > 2.0f ||
       fabs(imuGetState().pitchRateDps) > 2.0f)
    {
        delay(10);
    }
    Serial.println("Calibrating body pose...");
    
    delay(2000);  // let robot settle

    for(int i = 0; i < samples; i++)
    {
        imuUpdate(CONTROL_DT);
        ImuState imu = imuGetState();

        rollSum += imu.rollDeg;
        pitchSum += imu.pitchDeg;

        delay(10);
    }

    rollSetpoint  = rollSum / samples;
    pitchSetpoint = pitchSum / samples;

    Serial.print("Roll Setpoint: ");
    Serial.println(rollSetpoint);

    Serial.print("Pitch Setpoint: ");
    Serial.println(pitchSetpoint);
}

void stabilizationInit()
{
    rollPID.reset();
    pitchPID.reset();

    calibrateBodyPose();
}

void stabilizationSetConfig(float kpRoll,
                            float kiRoll,
                            float kdRoll,
                            float kpPitch,
                            float kiPitch,
                            float kdPitch,
                            float rollDeadband,
                            float pitchDeadband,
                            float maxRollCorr,
                            float maxPitchCorr)
{
    STABILIZATION_LOCK();

    KP_ROLL = kpRoll;
    KI_ROLL = kiRoll;
    KD_ROLL = kdRoll;

    KP_PITCH = kpPitch;
    KI_PITCH = kiPitch;
    KD_PITCH = kdPitch;

    ROLL_DEADBAND = rollDeadband;
    PITCH_DEADBAND = pitchDeadband;
    MAX_ROLL_CORR = maxRollCorr;
    MAX_PITCH_CORR = maxPitchCorr;

    rollPID.setTunings(KP_ROLL, KI_ROLL, KD_ROLL);
    pitchPID.setTunings(KP_PITCH, KI_PITCH, KD_PITCH);

    rollPID.reset();
    pitchPID.reset();

    STABILIZATION_UNLOCK();
}

void updateBodyCompensation()
{
    ImuState imu = imuGetState();
    
    DEBUG_PRINT("Roll: ");
    DEBUG_PRINT(imu.rollDeg);

    DEBUG_PRINT(" Pitch: ");
    DEBUG_PRINTLN(imu.pitchDeg);

    float rollMeasurement = imu.rollDeg;
    float pitchMeasurement = imu.pitchDeg;


    if(fabs(rollSetpoint - rollMeasurement) < ROLL_DEADBAND)
    rollMeasurement = rollSetpoint;

    if(fabs(pitchSetpoint - pitchMeasurement) < PITCH_DEADBAND)
    pitchMeasurement = pitchSetpoint;

    STABILIZATION_LOCK();

    float rollCorr = rollPID.update(
        rollSetpoint,
        rollMeasurement,
        CONTROL_DT);

    float pitchCorr = pitchPID.update(
        pitchSetpoint,
        pitchMeasurement,
        CONTROL_DT);

    STABILIZATION_UNLOCK();

    rollCorr = constrain(
    rollCorr,
    -MAX_ROLL_CORR,
     MAX_ROLL_CORR);

    pitchCorr = constrain(
    pitchCorr,
    -MAX_PITCH_CORR,
     MAX_PITCH_CORR);

    rollCompLeft  = rollCorr;
    rollCompRight = -rollCorr;

    pitchCompFront = -pitchCorr;
    pitchCompRear  = pitchCorr;
}
