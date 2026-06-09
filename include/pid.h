#pragma once

class PID
{
public:
    PID(float kp = 0.0f,
        float ki = 0.0f,
        float kd = 0.0f);

    float update(float setpoint,
                 float measurement,
                 float dt);

    void reset();

    void setTunings(float kp,
                    float ki,
                    float kd);

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _prevError;
};