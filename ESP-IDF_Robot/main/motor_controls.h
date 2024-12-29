#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLSC_H

// Interpolate value (x) based on raw reading, min/max limits.
/*

    8191     4095
    4095        0
    0       -4095
*/
static int interpolate_raw_val (int min, int max, int raw) {
    int x;

    x = raw - 8191;

    return x;
}

#endif