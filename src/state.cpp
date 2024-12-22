#include "state.h"

namespace STATE{
    
    int8_t setpoint_pitch=0;
    int8_t setpoint_roll=0;

    int sp_iterator = 0;

    

    
    void stateTask(void *parameter){
        while(1)
        {
            CALIB::grid_calib();
            while(1);
            
            // sp_iterator = (sp_iterator+1)%6;
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }
}