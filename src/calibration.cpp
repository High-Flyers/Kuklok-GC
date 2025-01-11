#include "calibration.h"



namespace CALIB
{
    // uint16_t calib_grid_roll[CALIB_GRID_SIZE][CALIB_GRID_SIZE];
    uint16_t calib_grid_roll[CALIB_GRID_SIZE][CALIB_GRID_SIZE] = {  //[roll][pitch]
        {2353, 2368, 2321, 2240, 2128, 2016, 1888, 1744, 1584},
        {2500, 2480, 2449, 2352, 2257, 2129, 2000, 1776, 1712},
        {2593, 2593, 2576, 2465, 2384, 2241, 2096, 1985, 1824},
        {2721, 2704, 2673, 2593, 2497, 2353, 2241, 2081, 1937},
        {2864, 2784, 2768, 2737, 2593, 2496, 2336, 2193, 1968},
        {2977, 2976, 2929, 2881, 2736, 2641, 2449, 2273, 2049},
        {3121, 3136, 3040, 2961, 2896, 2737, 2592, 2368, 2112},
        {3232, 3232, 3200, 3121, 2977, 2848, 2624, 2464, 2192},
        {3360, 3329, 3312, 3233, 3153, 2912, 2752, 2576, 2353}
    };
    // uint16_t calib_list_pitch[CALIB_GRID_SIZE];
    uint16_t calib_list_pitch[CALIB_GRID_SIZE] = {3312, 3185, 2961, 2801, 2560, 2385, 2160, 1968, 1680};

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

        Serial.print("avg_diff:");
        Serial.println(avg_dif);
        Serial.println("Cleand up:");
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

    //Calculates gimbal pitch and roll angles based on actuator positions
    bool get_pitch_roll(int ap, int ar, float &pitch, float &roll)
    {
        int8_t pitch_range = -1;
        for (uint8_t i = 0; i < CALIB_GRID_SIZE-1; i++)
        {
            if(calib_list_pitch[i] >= ap && calib_list_pitch[i+1] < ap)
            {
                pitch_range = i;
                break;
            }
        }
        
        if(pitch_range == -1) return false;

        float pitch_ratio = 1.0*(ap-calib_list_pitch[pitch_range]) / (calib_list_pitch[pitch_range+1]-calib_list_pitch[pitch_range]);
        pitch = (1.0*(pitch_range+pitch_ratio)*CALIB_STEP)-CALIB_RANGE;

        int8_t roll_range = -1;
        float roll_p1, roll_p2;
        for (uint8_t i = 0; i < CALIB_GRID_SIZE-1; i++)
        {
            roll_p1 = calib_grid_roll[i][pitch_range] + (pitch_ratio*(calib_grid_roll[i][pitch_range+1]-calib_grid_roll[i][pitch_range]));
            roll_p2 = calib_grid_roll[i+1][pitch_range] + (pitch_ratio*(calib_grid_roll[i+1][pitch_range+1]-calib_grid_roll[i+1][pitch_range]));
            if(roll_p1 >= ar && roll_p2 < ar ||
                roll_p1 <= ar && roll_p2 > ar)
            {
                roll_range = i;
                break;
            }
        }

        if(roll_range == -1) return false;

        float roll_ratio = (ar - roll_p1) / (roll_p2-roll_p1);
        roll = (1.0*(roll_range+roll_ratio)*CALIB_STEP)-CALIB_RANGE;

        return true;
        
    }

    
    void get_roll_gradients(float pitch, float roll, float &gp, float &gr)
    {
        int pitch_range = (int)round((pitch+CALIB_RANGE)/CALIB_STEP);
        int roll_range = (int)round((roll+CALIB_RANGE)/CALIB_STEP);

        if(pitch_range == 0)
        {
            gp = 1.0*(calib_grid_roll[roll_range][0] - calib_grid_roll[roll_range][1])/CALIB_STEP;
        }else if(pitch_range == CALIB_GRID_SIZE-1)
        {
            gp = 1.0*(calib_grid_roll[roll_range][CALIB_GRID_SIZE-2] - calib_grid_roll[roll_range][CALIB_GRID_SIZE-1])/CALIB_STEP;
        }else
        {
            gp = 1.0*(calib_grid_roll[roll_range][pitch_range-1] - calib_grid_roll[roll_range][pitch_range+1])/(CALIB_STEP*2);
        }

        if(roll_range == 0)
        {
            gr = 1.0*(calib_grid_roll[1][pitch_range] - calib_grid_roll[0][pitch_range])/CALIB_STEP;
        }else if(roll_range == CALIB_GRID_SIZE-1)
        {
            gr = 1.0*(calib_grid_roll[CALIB_GRID_SIZE-1][pitch_range] - calib_grid_roll[CALIB_GRID_SIZE-2][pitch_range])/CALIB_STEP;
        }else
        {
            gr = 1.0*(calib_grid_roll[roll_range+1][pitch_range] - calib_grid_roll[roll_range-1][pitch_range])/(CALIB_STEP*2);
        }
    }
    
    void get_pitch_gradient(float pitch, float &gp)
    {
        int pitch_range = (int)round((pitch+CALIB_RANGE)/CALIB_STEP);
        
        if(pitch_range == 0)
        {
            gp = 1.0*(calib_list_pitch[0] - calib_list_pitch[1])/CALIB_STEP;
        }else if(pitch_range == CALIB_GRID_SIZE-1)
        {
            gp = 1.0*(calib_list_pitch[CALIB_GRID_SIZE-2] - calib_list_pitch[CALIB_GRID_SIZE-1])/CALIB_STEP;
        }else
        {
            gp = 1.0*(calib_list_pitch[pitch_range-1] - calib_list_pitch[pitch_range+1])/(CALIB_STEP*2);
        }
    }

    int sign(int val)
    {
        if (val < 0) return -1;
        if (val==0) return 0;
        return 1;
    }

    void get_actuator_velocities(float ap, float ar, float pitch_rate, float roll_rate, int &pitch_act_vel, int &roll_act_vel)
    {
        float pitch, roll;
        get_pitch_roll(ap,ar,pitch,roll);

        float pitch_gp, roll_gp, roll_gr;
        get_roll_gradients(pitch,roll,roll_gp,roll_gr);
        get_pitch_gradient(pitch, pitch_gp);

        pitch_act_vel = (int) (pitch_rate * pitch_gp);
        // roll_act_vel = (int) (roll_rate*roll_gr);
        roll_act_vel = (int) (sign(roll_rate) * sqrt((pitch_rate*roll_gp * pitch_rate*roll_gp) + (roll_rate*roll_gr * roll_rate*roll_gr)));
    }
}