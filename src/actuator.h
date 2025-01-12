#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>

#define SERVO_DEADZONE 22
#define SERVO_SILENTZONE 3

class Actuator{

    uint8_t _pin_a;
    uint8_t _pin_b;

    uint8_t _out_pwm;

public:
    void init(uint8_t pin_a, uint8_t pin_b);
    void set_vel(int16_t vel);

private:
    void _set_pwm(uint8_t pwm_a, uint8_t pwm_b);

};







#endif