#pragma once

struct FootPosition {
    float x;
    float y;
    float z;
};

FootPosition computeFK(float shoulderDeg,
                       float hipDeg,
                       float kneeDeg);