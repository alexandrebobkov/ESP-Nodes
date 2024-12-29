#ifndef CONTROLS_H
#define CONTROLSC_H

/* 

To prevent assigning forbidden rpm values to the motor (i.e. to avoid short-citcuit) accidentially,
we define one struct tha tholds RPMs for four motors, as opposed to defining array of structs for single motor.

*/ 
struct motors_rpm {
    int8_t motor1_rpm;
    int8_t motor2_rpm;
    int8_t motor3_rpm;
    int8_t motor4_rpm;
};

#endif