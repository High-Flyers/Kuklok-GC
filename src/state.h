#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "calibration.h"

namespace STATE{

    extern int8_t setpoint_pitch;
    extern int8_t setpoint_roll;

    void stateTask(void *parameter);

}




#endif