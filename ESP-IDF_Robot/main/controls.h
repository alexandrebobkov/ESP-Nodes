#ifndef CONTROLS_H
#define CONTROLSC_H

/* 

To prevent assigning forbidden rpm values to the motor (i.e. to avoid short-citcuit) accidentially,
we define one struct tha tholds RPMs for four motors, as opposed to defining array of structs for single motor.

*/ 

/*

Struct that holds PCM for RPMs for each of 4 motors.
Positive PCM values for clock-wise rotation, and negative values for counter-vise rotation.

*/
struct motors_rpm {
    int* motor1_rpm_pcm;
    int* motor2_rpm_pcm;
    int* motor3_rpm_pcm;
    int* motor4_rpm_pcm;
};

static void motors_rpm_init (void);

#endif