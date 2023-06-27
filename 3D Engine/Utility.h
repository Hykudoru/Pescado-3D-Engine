#pragma once
#ifndef UTILITY_H
#define UTILITY_H
extern bool DEBUGGING;
#define List std::vector
const float PI = 3.14159265359f;

float Clamp(float value, float min, float max)
{
    if (value < min) {
        value = min;
    }
    else if (value > max) {
        value = max;
    }
    return value;
}

float ToDeg(float rad) {
    return rad * 180.0 / PI;
}

float ToRad(float deg) {
    return deg * PI / 180.0;
}

struct Range
{
    float min;
    float max;

    Range(float minimum, float maximum)
    {
        min = minimum;
        max = maximum;
    }
};

Range ProjectVertsOntoAxis(const Vec3 verts[], int count, Vec3& axis)
{
    float min = 0;
    float max = 0;
    for (size_t k = 0; k < count; k++)
    {
        float dist = DotProduct(verts[k], axis);
        if (k == 0)
        {
            min = dist;
            max = dist;
        }
        else if (dist < min) {
            min = dist;
        }
        else if (dist > max) {
            max = dist;
        }
    }

    return Range(min, max);
}
#endif