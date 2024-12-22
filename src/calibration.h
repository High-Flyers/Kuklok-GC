#ifndef CALIB_H
#define CALIB_H

#include <Arduino.h>
#include "state.h"

#define CALIB_RANGE         20          //calib grid is set to <-CALIB_RANGE, CALIB_RANGE>
#define CALIB_DELAY_ROLL    3000 / portTICK_PERIOD_MS
#define CALIB_DELAY_PITCH   5000 / portTICK_PERIOD_MS
#define CALIB_STEP          5
#define CALIB_CLEANUP_TRES  2

#define CALIB_GRID_SIZE     (CALIB_RANGE / CALIB_STEP * 2) + 1

namespace CALIB
{
    void grid_calib();
    void cleanup_grid();
    bool get_pitch_roll(int ap, int ar, float &pitch, float &roll);
}





#endif