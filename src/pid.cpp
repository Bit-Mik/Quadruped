#include "pid.h"

PID::PID(float kp,
         float ki,
         float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _integral = 0.0f;
    _prevError = 0.0f;
}

float PID::update(float setpoint,
                  float measurement,
                  float dt)
{
    float error = setpoint - measurement;

    _integral += error * dt;

    float derivative = 0.0f;

    if(dt > 0.0f)
    {
        derivative =
            (error - _prevError) / dt;
    }

    _prevError = error;

    return
        _kp * error +
        _ki * _integral +
        _kd * derivative;
}

void PID::reset()
{
    _integral = 0.0f;
    _prevError = 0.0f;
}

void PID::setTunings(float kp,
                     float ki,
                     float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}