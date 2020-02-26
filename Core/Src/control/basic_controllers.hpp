#pragma once
#include "../signal_processing/math_functions.hpp"

namespace ast { namespace control {

template <typename T>
class PiController
{

    T u0 = 0;
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float Ts = 0.01f;

    PiController()
    {
        u0 = 0;
    }

    T operator()(T xd, T x)
    {
        T e = xd - x;
        T u = kp * ki * Ts * e + u0;
        u0 = u;
        return u + kp * e;
    }
};

// PI with anti wind-up correction
template <typename T>
class PiSatController
{
    T u0;
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float ks = 0.0f;
    float um = 1e-4f;

    float Ts = 0.01f;

    T e;


    PiSatController()
    {
        u0 = 0;
    }

    T operator()(T xd, T x)
    {
        e = xd - x;
        T uc = kp * e + u0;

        T u = sat(uc, -um, um);
        T u_cor = ks * (u - uc);
        u0 += Ts * (kp * ki * e + u_cor);

        return u;
    }
};

// PI with anti wind-up correction and a simple static feed-forward term
template <typename T>
class PiSatFFController
{
public:
    T u0 = 0;

    float kp = 0.0f;
    float ki = 0.0f;
    float ks = 0.0f;
    float um = 1e-4f;

    float fks = 0;

    float Ts = 0.01f;

    T operator()(T xd, T x)
    {
        T e = xd - x;
        T uc = kp * e + u0;

        T u = sat(uc, -um, um);
        T u_cor = ks * (u - uc);

        //if ((u0 < um) && (u0 > -um))
        u0 += Ts * (kp * ki * e + u_cor);

        return sat(u + fks * xd, -um, um);
    }
};


class AdrcFirstOrderController
{
    AdrcFirstOrderController();



};

class StateFeedbackSOS
{
public:
    float wn;
    float zeta;
    float um;
    float k1;
    float k2;

    StateFeedbackSOS(float wn, float zeta)
    {
        this->wn = wn;
        this->zeta = zeta;
        this->um = um;
        Initialize();
    }

    void Initialize() {
        k1 = wn*wn;
        k2 = 2*wn*zeta;
    }

    float operator()(double posError, double velError)
    {
        float u = k1 * posError + k2 * velError;
        return u; //sat<float>(u, -um, um);;
    }
};

class ErrorStateFeedbackSOS
{
public:
    float wn;
    float zeta;
    float um;
    float k1;
    float k2;

    ErrorStateFeedbackSOS(float wn, float zeta, float um)
    {
        this->wn = wn;
        this->zeta = zeta;
        this->um = um;
        Initialize();
    }

    void Initialize() {
        k1 = wn*wn;
        k2 = 2*wn*zeta;
    }

    float operator()(double posError, double velError)
    {
        float u = - k1 * posError - k2 * velError;
        return sat<float>(u, -um, um);;
    }
};



class StateHomogenousFeedbackSOS
{
    float alpha1, alpha2;
public:
    float k1;
    float k2;
    float um;


    StateHomogenousFeedbackSOS(float wn, float zeta, float alpha)
    {
        updateGains(wn, zeta, alpha);
    }

    void updateGains(float wn, float zeta, float alpha)
    {
        k1 = wn*wn;
        k2 = 3*wn*zeta;;
        this->um = um;
        this->alpha1 = alpha;
        alpha2 = 2*alpha/(1+alpha);
        this->um = um;
    }

    void updateGains(float wn, float zeta)
    {
        k1 = wn*wn;
        k2 = 3*wn*zeta;;
    }

    float operator()(double posError, double velError)
    {
        //float u = k1 * fractionRoot(posError, alpha1) + k2 * fractionRoot(velError, alpha2);
        float u = k1 * fractionRootApprox(posError, alpha1) + k2 * fractionRootApprox(velError, alpha2);
        return u;
    }
};

}
}
