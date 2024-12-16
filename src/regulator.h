#ifndef REGULATOR_H
#define REGULATOR_H

#include "Arduino.h"


class Regulator
{
    float _p, _i, _d;

    float _err_sum, _last_err;

public:
    Regulator();
    Regulator(float P, float I, float D);

    int regulate_PID(int setpoint, int y, int last_loop_time);
};





#endif