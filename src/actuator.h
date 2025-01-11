#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>

#define SERVO_DEADZONE 25
#define SERVO_SILENTZONE 5

class Actuator{

    uint8_t _pin_a;
    uint8_t _pin_b;

    uint8_t _out_pwm;

public:
    void init(uint8_t pin_a, uint8_t pin_b);
    void set_vel(uint8_t vel); //0-200 -> 100 is idle
    void set_vel_new(int vel); //0-200 -> 100 is idle

private:
    void _set_pwm(uint8_t pwm_a, uint8_t pwm_b);

};







#endif