#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#include "imu.h"
#include "regulator.h"
#include "actuator.h"
#include "state.h"

#define LOOP_FREQ 100 //Hz

#define PIN_MOT_1_A 17 //silnik 1 przód
#define PIN_MOT_1_B 19 //silnik 1 przód
#define PIN_MOT_2_A 20 //silnik 2 przód
#define PIN_MOT_2_B 18 //silnik 2 tył



Madgwick filter;
Regulator reg_roll(2,4,0.15); //2,4,0.15
Regulator reg_pitch(2,4,0.15);
Actuator servo_roll;
Actuator servo_pitch;
unsigned long microsPerReading, microsPrevious;


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

  CALIB::cleanup_grid();
}

void loop() {

  if (micros() - microsPrevious >= microsPerReading) {
    unsigned long micros_start = micros();

    uint16_t angle_roll = analogRead(0);
    uint16_t angle_pitch = analogRead(1);
    
    CALIB::get_pitch_roll(angle_pitch, angle_roll, STATE::gimbal_pitch_angle, STATE::gimbal_roll_angle);
    
    int ur, up;

    IMU::read();

    filter.updateIMU(IMU::gyro_x, IMU::gyro_y, IMU::gyro_z, IMU::acc_x, IMU::acc_y, IMU::acc_z);

    ur = reg_roll.regulate_PID(STATE::setpoint_roll,filter.getRoll(),LOOP_FREQ);
    up = reg_pitch.regulate_PID(STATE::setpoint_pitch,filter.getPitch(),LOOP_FREQ);

    if(angle_roll > 3500 || angle_roll < 2000)
    {
      servo_roll.set_vel(0);
    }
    if(angle_pitch > 3600 || angle_roll < 1800)
    {
      servo_pitch.set_vel(0);
    }

    servo_roll.set_vel(ur);
    servo_pitch.set_vel(up);

    Serial.printf("$%f %f %d %d %f %f;", filter.getPitch(), filter.getRoll(), angle_pitch, angle_roll, STATE::gimbal_pitch_angle, STATE::gimbal_roll_angle);
    // Serial.printf("$%f %f %d %d;\n", 10*filter.getPitch(), 10*filter.getRoll(),angle_pitch, angle_roll);
    // Serial.printf("$%f %f %f %d %d;", 10*filter.getRoll(), filter.getPitch(), filter.getYaw(), u, uu);
    // Serial.printf("$%d;", angle);
    // Serial.println(micros() - micros_start);


    microsPrevious = microsPrevious + microsPerReading;
  }
}
