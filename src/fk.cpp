#include <math.h>
#include "config.h"

struct FootPosition
{
    float x;
    float y;
    float z;
};

FootPosition computeFK(
    float shoulderDeg,
    float hipDeg,
    float kneeDeg)
{
    FootPosition p;

    float t1 = radians(shoulderDeg);
    float t2 = radians(hipDeg);
    float t3 = radians(kneeDeg);

    // Hip-knee planar reach

    float x_plane =
        UPPER_LEG_LENGTH * sin(t2) +
        LOWER_LEG_LENGTH * sin(t2 + t3);

    float z_plane =
       -(UPPER_LEG_LENGTH * cos(t2) +
         LOWER_LEG_LENGTH * cos(t2 + t3));

    // Shoulder rotation

    float y =
        SHOULDER_LENGTH +
        SHOULDER_WIDTH * sin(t1);

    float z =
        z_plane -
        SHOULDER_WIDTH * cos(t1);

    p.x = x_plane;
    p.y = y - SHOULDER_LENGTH;
    p.z = z;

    return p;
}