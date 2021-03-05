#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cstdio>

class PIDContorller
{

private:
    float err = 0;      //定義偏差值
    float err_last = 0; //定義上一個偏差值
    float output_last = 0;
    float maxValue = 0, minValue = 0, maxStep = 0;
    int constrain_flag = 0;
    int step_flag = 0;

public:
    float Kp, Ki, Kd;   //定義比例、積分、微分系數
    float integral = 0; //定義積分值

    PIDContorller();
    PIDContorller(float _Kp, float _Ki, float _Kd, float _max, float _min, float _maxStep);
    ~PIDContorller();
    float calculate(float target, float actual);
    float calculate(float error);
    void clear();
};

#endif
