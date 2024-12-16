#include "imu.h"



namespace IMU{

    LSM6DSO imu;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    void begin()
    {
        if( imu.begin() )
            Serial.println("Ready.");
        else { 
            Serial.println("Could not connect to IMU.");
            Serial.println("Freezing");
            while(1);
        }

        if( imu.initialize(BASIC_SETTINGS) )
            Serial.println("Loaded Settings.");

        imu.setAccelDataRate(208);   
        imu.setGyroDataRate(208);
        imu.setGyroRange(2000);

        delay(100); // Wait for sensor to stabilize
    }

    void read()
    {
        acc_x = imu.readFloatAccelX();
        acc_y = imu.readFloatAccelY();
        acc_z = imu.readFloatAccelZ();

        gyro_x = imu.readFloatGyroX();
        gyro_y = imu.readFloatGyroY();
        gyro_z = imu.readFloatGyroZ();
    }
}