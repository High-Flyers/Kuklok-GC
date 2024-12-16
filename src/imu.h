#ifndef IMU_H
#define IMU_H


#include "SparkFunLSM6DSO.h"

namespace IMU
{
    extern float acc_x, acc_y, acc_z;
    extern float gyro_x, gyro_y, gyro_z;

    void begin();
    void read();
}



#endif