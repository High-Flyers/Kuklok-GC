#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#include "imu.h"
#include "regulator.h"
#include "actuator.h"
#include "state.h"

#define LOOP_FREQ 200 //Hz

#define PIN_MOT_1_A 17 //silnik 1 przód
#define PIN_MOT_1_B 19 //silnik 1 przód
#define PIN_MOT_2_A 20 //silnik 2 przód
#define PIN_MOT_2_B 18 //silnik 2 tył

#define MAX_ROLL_ANGLE 18
#define MAX_PITCH_ANGLE 18

Madgwick filter;
Regulator reg_roll(0.7,10,0); //2,4,0.15
Regulator reg_pitch(0.7,10,0);
Actuator servo_roll;
Actuator servo_pitch;
unsigned long microsPerReading, microsPrevious;

void constrain_to_angles(float &, float &);

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000UL); // 400kHz

  
  IMU::begin();
  filter.begin(LOOP_FREQ);

  servo_roll.init(PIN_MOT_1_A, PIN_MOT_1_B);
  servo_pitch.init(PIN_MOT_2_B, PIN_MOT_2_A);
  
  microsPerReading = 1000000 / LOOP_FREQ;
  microsPrevious = micros();

  xTaskCreatePinnedToCore(STATE::stateTask, "State Task", 2048, NULL, 10, NULL, 0);

  delay(8000);
  Serial.println("SETUP DONE");

}

void loop() {

  if (micros() - microsPrevious >= microsPerReading) {
    unsigned long micros_start = micros();

    uint16_t angle_roll = analogRead(0);
    uint16_t angle_pitch = analogRead(1);
    
    CALIB::get_pitch_roll(angle_pitch, angle_roll, STATE::gimbal_pitch_angle, STATE::gimbal_roll_angle);
    
    float u_r, u_p;
    int act_r, act_p;

    IMU::read();

    filter.updateIMU(IMU::gyro_x, IMU::gyro_y, IMU::gyro_z, IMU::acc_x, IMU::acc_y, IMU::acc_z);

    u_r = reg_roll.regulate_PID((float)STATE::setpoint_roll,filter.getRoll(),LOOP_FREQ);
    u_p = reg_pitch.regulate_PID((float)STATE::setpoint_pitch,filter.getPitch(),LOOP_FREQ);

    constrain_to_angles(u_r, u_p);

    CALIB::get_actuator_velocities(angle_pitch,angle_roll,u_p,u_r,act_p,act_r);

    act_p /=20;
    act_r /=20;

    servo_roll.set_vel(act_r);
    servo_pitch.set_vel(act_p);

    Serial.printf("$%f %f %d %d;", filter.getPitch(), filter.getRoll(), act_p, act_r);


    microsPrevious = microsPrevious + microsPerReading;
  }
}

void constrain_to_angles(float &roll, float &pitch)
{
    if(STATE::gimbal_roll_angle > MAX_ROLL_ANGLE)
    {
      roll = min(roll,0.0f);
    }
    if(STATE::gimbal_roll_angle < -MAX_ROLL_ANGLE)
    {
      roll = max(roll,0.0f);
    }
    if(STATE::gimbal_pitch_angle > MAX_PITCH_ANGLE)
    {
      pitch = min(pitch,0.0f);
    }
    if(STATE::gimbal_pitch_angle < -MAX_PITCH_ANGLE)
    {
      pitch = max(pitch,0.0f);
    }
}
