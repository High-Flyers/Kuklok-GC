#include "regulator.h"

#define MAX_REG_OUTPUT 30

Regulator::Regulator()
{
    _p = 2;
    _i = 20;
    _d = 0;
}

Regulator::Regulator(float P, float I, float D)
{
    _p = P;
    _i = I;
    _d = D;
}

int Regulator::regulate_PID(int setpoint, int y, int last_loop_time)
{
    float err = setpoint - y;
    float p_term = err;

    float i_term = 0;
    if(_i>0)
    {
        _err_sum += err;
        i_term = (1.0*last_loop_time/1000/_i * _err_sum);
    }

    float d_term = 0;
    if(_d>0)
    {
        d_term = _d*(_last_err - err);
        _last_err = err;
    }

    //Regulator has a bias of 100 witch helps with response times itp.
    //It is not something normal PIDs have, but it works for me.
    float u = 100 + _p*(p_term + i_term + d_term);

    u = constrain(u, 0, 200);

    if(_i>0 && (u == 0 || u == 200)) //anti-windup
        _err_sum -= err;

    return u;
}


float Regulator::regulate_PID(float setpoint, float y, int last_loop_time)
{
    float err = setpoint - y;
    float p_term = err;

    float i_term = 0;
    if(_i>0)
    {
        _err_sum += err;
        i_term = (1.0*last_loop_time/1000/_i * _err_sum);
    }

    float d_term = 0;
    if(_d>0)
    {
        d_term = _d*(_last_err - err);
    }
    _last_err = err;

    float u = _p*(p_term + i_term + d_term);

    u = constrain(u, -MAX_REG_OUTPUT, MAX_REG_OUTPUT);

    if(_i>0 && (u == -MAX_REG_OUTPUT || u == MAX_REG_OUTPUT)) //anti-windup
        _err_sum -= err;

    return u;
}

void Regulator::skip_integration()
{
    if(_i>0)
        _err_sum-= _last_err;
}