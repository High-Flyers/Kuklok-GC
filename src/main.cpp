#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#include "imu.h"
#include "regulator.h"

#define LOOP_FREQ 100 //Hz

#define PIN_MOT_1_A 17 //silnik 1 przód
#define PIN_MOT_1_B 19 //silnik 1 tył

Madgwick filter;
Regulator reg1(4,8,0.2);
unsigned long microsPerReading, microsPrevious;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000UL); // 400kHz
  
  IMU::begin();
  filter.begin(LOOP_FREQ);
  
  microsPerReading = 1000000 / LOOP_FREQ;
  microsPrevious = micros();
  
  pinMode(PIN_MOT_1_A, OUTPUT);
  pinMode(PIN_MOT_1_B, OUTPUT);
  analogReadResolution(12);
  analogWriteFrequency(PIN_MOT_1_A, 500);
  
  delay(2000);
  return;

  int pwm_val = 200;
  float vels[25];
  while(1){

    uint16_t angle = analogRead(0);
    unsigned long start_time = millis();
    
    while(angle>2500){
      analogWrite(PIN_MOT_1_A,pwm_val);
      analogWrite(PIN_MOT_1_B,0);
      delay(1);
      angle = analogRead(0);

      if(angle > 4000 || angle < 2000){
        analogWrite(PIN_MOT_1_A,0);
        analogWrite(PIN_MOT_1_B,0);
        return;
      }
    }
    while(angle<3500){
      analogWrite(PIN_MOT_1_A,0);
      analogWrite(PIN_MOT_1_B,pwm_val);
      delay(1);
      angle = analogRead(0);

      if(angle > 4000 || angle < 2000){
        analogWrite(PIN_MOT_1_A,0);
        analogWrite(PIN_MOT_1_B,0);
        return;
      }
    }
    analogWrite(PIN_MOT_1_A,0);
    analogWrite(PIN_MOT_1_B,0);

    int duration = millis()-start_time;
    float velocity = 1000.0/duration;

    if(pwm_val <=175 && pwm_val >=50)
      vels[(pwm_val-50)/5]=velocity;

    Serial.printf("%d: %d \n", pwm_val, duration);

    if(duration > 1000*2 || pwm_val <=50)
      break;

    pwm_val -= 5;


  }


  for (int i = 0; i < 25; i++)
  {
    Serial.printf("%d: %f \n", i, vels[i]);
  }
  

}

void loop() {

  if (micros() - microsPrevious >= microsPerReading) {
    unsigned long micros_start = micros();
    uint16_t angle = analogRead(0);
    int u;

    IMU::read();

    filter.updateIMU(IMU::gyro_x, IMU::gyro_y, IMU::gyro_z, IMU::acc_x, IMU::acc_y, IMU::acc_z);

    
    if(angle > 3500 || angle < 2000)
    {
      analogWrite(PIN_MOT_1_A,0);
      analogWrite(PIN_MOT_1_B,0);
    }else
    {
      u = reg1.regulate_PID(0,filter.getRoll(),LOOP_FREQ);

      if(u > 95 && u < 105)
      {
        analogWrite(PIN_MOT_1_A,0);
        analogWrite(PIN_MOT_1_B,0);
      }else
      {
        analogWrite(PIN_MOT_1_A,max(0,-min(0,u-150)));
        analogWrite(PIN_MOT_1_B,max(0,max(0,u-50)));
      }
    }

    Serial.printf("$%f %f %f %d;", 10*filter.getRoll(), filter.getPitch(), filter.getYaw(), u);
    // Serial.printf("$%d;", angle);
    // Serial.println(micros() - micros_start);

    microsPrevious = microsPrevious + microsPerReading;
  }
}
