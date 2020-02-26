#pragma once
#include <math.h>
#include <stdint.h>

#define RAD2ARC (180/M_PI*3600)
#define DEG2RAD (M_PI/180)
#define NOMINALEARTHVEL (2*M_PI/86164.09)

inline float subtractAngles(float phi1, float phi2)
{
    auto error = phi1 - phi2;
    if (error < -M_PI)
        error += 2*M_PI;
    else if (error > M_PI)
        error -= 2*M_PI;

    return error;
}

inline float extendAngleRange(float phi, float phiOld, float offset)
{
    auto e = phi - phiOld;

    if (e < -M_PI)
        offset += 2*M_PI;
    else if (e > M_PI)
        offset -= 2*M_PI;

    return offset;
}

inline float correctAngle(float phi)
{
    if (phi < -M_PI)
        phi += 2*M_PI;
    else if (phi > M_PI)
        phi -= 2*M_PI;

    return phi;
}

inline int compareTwoIntegers(const void * a, const void * b)
{
    uint32_t _a = *(uint32_t*)a;
    uint32_t _b = *(uint32_t*)b;
    if(_a < _b) return -1;
    else if(_a == _b) return 0;
    else return 1;
}

template <typename T>
inline T sat(T in, T low, T high)
{
    T out = in;
    if (in < low)
        out = low;
    else if (in > high)
        out = high;
    //return fmin(fmax(in, low), high);
    return out;
}

template <typename T>
inline T linearWeight(T in, T low, T high)
{
    const T a = 1.0/(high-low);
    if (in < low)
        return 0;
    if (in > high)
        return 1;

    return (a*(in-low));
}

inline float fractionRoot(float x, float a)
{
    if (x > 0)
        return powf(x, a);
    else
        return -powf(-x, a);
}

inline float fractionRootApproxA(float x, float a)
{

    constexpr float e = 4e-6;

    bool negative = false;
    if (x < 0)
    {
        negative = true;
        x = -x;
    }

    float y;
    if (x > e)
        y = powf(x, a);
    else
        y = powf(e, a-1)*x;

    if (negative)
        y = -y;

    return y;
}


inline float fractionRootApprox(float x, const float a)
{

    constexpr float e = 0e-8;
    const float fe = powf(e, a);

    bool negative = false;
    if (x < 0)
    {
        negative = true;
        x = -x;
    }

    float y;

    y = powf(x+e, a)-fe;

    if (negative)
        y = -y;

    return y;
}



inline float sign(float x)
{
    if (x==0)
        return 0;
    if (x > 0)
        return -1.0f;
    else
        return 1.0f;
}

inline bool isInRange(double x, double xmin, double xmax)
{
    return (x <= xmax) && (x >= xmin);
}

inline double averageAngles(double * phi, uint8_t n)
{
    auto res = phi[0];

    // make correction
    for(int i = 1; i < n; i++)
    {
        auto e = phi[0]-phi[i];
        if (fabs(e) > M_PI)
            phi[i] -= sign(e)*2*M_PI;
        res += phi[i];
    }

    res /= n;

    if (res < 0)
        res += 2*M_PI;
    return res;
}

//positive-only modulo (x mod y)
inline int mod(int x, int y) {
    return ((x % y + y) % y);
}

inline double mod(double x, double y) {
    return fmod(fmod(x, y) + y, y);
}
