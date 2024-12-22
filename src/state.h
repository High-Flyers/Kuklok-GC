#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "calibration.h"

namespace STATE{

    extern int8_t setpoint_pitch;
    extern int8_t setpoint_roll;

    extern float gimbal_pitch_angle;
    extern float gimbal_roll_angle;

    void stateTask(void *parameter);

}




#endif