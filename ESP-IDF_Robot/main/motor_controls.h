#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLSC_H

// Interpolate value (x) based on raw reading, min/max limits.
/*

    Joystick scale:     4096    2048        0
    PWM scale:          8191    4096        0

    PWM Output:         +8191       0   -8191
*/
static int interpolate_raw_val (int raw) {
    int x;

    x = raw*2;

    return x;
}
// Function that converts raw value from joystick scale (0 to 4096) to PCM scale (-8192 to 8192).
static int rescale_raw_val (int raw) {

    int s;
    //s = 4*raw - 8940;
    //s = (8190/1200^1)*(raw - 1200)^1;   // Linear rescaling
    s = (8190/1200^3)*(raw - 1200)^3;   // Cubic rescaling
    return s;
}

#endif