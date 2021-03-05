#include "PIDController.h"

PIDContorller::PIDContorller() {}
PIDContorller::PIDContorller(float _Kp, float _Ki, float _Kd, float _max, float _min, float _maxStep)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;

    maxValue = _max;
    minValue = _min;
    maxStep = _maxStep;

    constrain_flag = 1;
    step_flag = 1;
}

PIDContorller::~PIDContorller()
{
}

float PIDContorller::calculate(float target, float actual)
{
    float output = 0;
    err = target - actual;
    integral += err;
    output = Kp * err + Ki * integral + Kd * (err - err_last);
    err_last = err;

    if (constrain_flag)
    {
        if (output > maxValue)
            output = maxValue;
        else if (output < minValue)
            output = minValue;
    }

    if (step_flag)
    {
        if ((output - output_last) > maxStep)
            output = output_last + maxStep;
        else if ((output - output_last) < -maxStep)
            output = output_last - maxStep;
    }

    output_last = output;

    return output;
}

float PIDContorller::calculate(float error)
{
    float output = 0;
    err = error;
    integral += err;
    output = Kp * err + Ki * integral + Kd * (err - err_last);
    err_last = err;

    if (constrain_flag)
    {
        if (output > maxValue)
        {
            output = maxValue;
            //integral -= err;
        }

        else if (output < minValue)
        {
            output = minValue;
           // integral -= err;
        }
    }

    if (step_flag)
    {
        if ((output - output_last) > maxStep)
            output = output_last + maxStep;
        else if ((output - output_last) < -maxStep)
            output = output_last - maxStep;
    }

    output_last = output;

    return output;
}

void PIDContorller::clear()
{
    err = err_last = integral = 0;
}