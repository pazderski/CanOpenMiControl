#pragma once
#include "math.h"

class SmoothEsoSOS
{
    const float Ts;

    float zeta = 0.8;
    float l1, l2, l3;

public:
    const float wn;
    float beta;

    float xi1, xi2, xi3;

    SmoothEsoSOS(float wn, float beta, float Ts) : Ts(Ts), wn(wn), beta(beta)
    {
        Initialize();
    }

    void updateBeta(float beta)
    {
        this->beta = beta;
    }

    void Initialize()
    {
        l1 = Ts*(3*zeta*wn);
        l2 = Ts*(1+2*zeta*zeta)*wn*wn;
        l3 = Ts*zeta*wn*wn*wn;

        xi1 = 0;
        xi2 = 0; xi3 = 0;
    }

    void InitializeEstimate(float pos)
    {
        xi1 = pos; xi2 = xi3 = 0;
    }

    void operator()(float pos, float u)
    {
        auto y = pos;
        // measurement error
        auto em = y - xi1;

        // prediction step
        xi1 += Ts * xi2;
        xi2 += Ts * xi3 + Ts * beta * u;

        // update step
        xi1 += l1 * em;
        xi2 += l2 * em;
        xi3 += l3 * em;
    }
};
