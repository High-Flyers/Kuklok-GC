#include "calibration.h"



namespace CALIB
{
    uint16_t calib_grid_roll[CALIB_GRID_SIZE][CALIB_GRID_SIZE];
    uint16_t calib_list_pitch[CALIB_GRID_SIZE];

    void step_up(int8_t &value)
    {
        value = value + CALIB_STEP;
    }

    void step_down(int8_t &value)
    {
        value = value - CALIB_STEP;
    }

    void grid_calib()
    {
        STATE::setpoint_pitch = 0;
        STATE::setpoint_roll = 0;

        vTaskDelay(CALIB_DELAY_ROLL);

        //jog to corner position
        while(STATE::setpoint_pitch >= -CALIB_RANGE)
        {
            step_down(STATE::setpoint_pitch);
            step_down(STATE::setpoint_roll);
            vTaskDelay(CALIB_DELAY_ROLL);
        }

        //zigzag grid
        while(STATE::setpoint_pitch <= CALIB_RANGE)
        {
            while(STATE::setpoint_roll < CALIB_RANGE)
            {
                step_up(STATE::setpoint_roll);
                vTaskDelay(CALIB_DELAY_ROLL);
                calib_grid_roll[(STATE::setpoint_roll+CALIB_RANGE)/CALIB_STEP][(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(0);
            }
            
            calib_list_pitch[(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(1);
            step_up(STATE::setpoint_pitch);
            vTaskDelay(CALIB_DELAY_PITCH);
            calib_grid_roll[(STATE::setpoint_roll+CALIB_RANGE)/CALIB_STEP][(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(0);

            while(STATE::setpoint_roll > -CALIB_RANGE)
            {
                step_down(STATE::setpoint_roll);
                vTaskDelay(CALIB_DELAY_ROLL);
                calib_grid_roll[(STATE::setpoint_roll+CALIB_RANGE)/CALIB_STEP][(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(0);
            }

            calib_list_pitch[(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(1);
            step_up(STATE::setpoint_pitch);
            vTaskDelay(CALIB_DELAY_PITCH);
            calib_grid_roll[(STATE::setpoint_roll+CALIB_RANGE)/CALIB_STEP][(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(0);
        }
        calib_list_pitch[(STATE::setpoint_pitch+CALIB_RANGE)/CALIB_STEP] = analogRead(1);

        for (int i = 0; i < CALIB_GRID_SIZE; i++)
        {
            for (int z = 0; z < CALIB_GRID_SIZE; z++)
            {
                Serial.print(calib_grid_roll[i][z]);
                Serial.print("; ");
            }
            Serial.println(" ");
        }

        for (int i = 0; i < CALIB_GRID_SIZE; i++)
        {
            Serial.print(calib_list_pitch[i]);
            Serial.print("; ");
        }
        Serial.println(" ");
        
    }

    void cleanup_grid()
    {
        float avg_dif = 0;
        for (int i = 0; i < CALIB_GRID_SIZE-1; i++)
        {
            for (int z = 0; z < CALIB_GRID_SIZE-1; z++)
            {
                avg_dif += abs(calib_grid_roll[i][z] - calib_grid_roll[i+1][z+1]);
            }
        }
        avg_dif /= CALIB_GRID_SIZE*CALIB_GRID_SIZE;


        for (int i = 0; i < CALIB_GRID_SIZE; i++)
        {
            for (int z = 1; z < CALIB_GRID_SIZE-1; z++)
            {
                if(abs(calib_grid_roll[i][z-1]-calib_grid_roll[i][z]) > CALIB_CLEANUP_TRES*avg_dif &&
                    abs(calib_grid_roll[i][z+1]-calib_grid_roll[i][z]) > CALIB_CLEANUP_TRES*avg_dif)
                    {
                        calib_grid_roll[i][z] = (calib_grid_roll[i][z-1]+calib_grid_roll[i][z+1])/2;
                    }
            }
        }
    }
}