#include "actuator.h"


void Actuator::init(uint8_t pin_a, uint8_t pin_b)
{
        _pin_a = pin_a;
        _pin_b = pin_b;
        
        pinMode(_pin_a, OUTPUT);
        pinMode(_pin_b, OUTPUT);
        analogReadResolution(12);
        analogWriteFrequency(_pin_a, 2000);
        analogWriteFrequency(_pin_b, 2000);
}

void Actuator::set_vel(uint8_t vel)
{
    if(vel > 100-SERVO_SILENTZONE && vel < 100+SERVO_SILENTZONE)
    {
        _set_pwm(0,0);
        return;
    }

    if(vel<100)
    {
        _out_pwm = 100-vel;
        _out_pwm+=SERVO_DEADZONE;
        _set_pwm(_out_pwm,0);
    }else
    {
        _out_pwm = vel-100;
        _out_pwm+=SERVO_DEADZONE;
        _set_pwm(0, _out_pwm);
    }
}

void Actuator::_set_pwm(uint8_t pwm_a, uint8_t pwm_b)
{
    analogWrite(_pin_a,pwm_a);
    analogWrite(_pin_b,pwm_b);
}
