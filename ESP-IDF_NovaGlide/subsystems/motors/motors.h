#ifndef MOTORS_H
#define MOTORS_H

#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"

// Motor configuration
#define MTR_FREQUENCY               7000
#define MTR_MODE                    LEDC_LOW_SPEED_MODE
#define MTR_DUTY_RES                LEDC_TIMER_13_BIT

// Motor GPIO pins
#define MTR_FRONT_LEFT_IO           6
#define MTR_FRONT_LEFT_TMR          LEDC_TIMER_0
#define MTR_FRONT_LEFT              LEDC_CHANNEL_1
#define MTR_FRONT_LEFT_DUTY         3361

#define MTR_FRONT_RIGHT_IO          5
#define MTR_FRONT_RIGHT_TMR         LEDC_TIMER_1
#define MTR_FRONT_RIGHT             LEDC_CHANNEL_0
#define MTR_FRONT_RIGHT_DUTY        3361

#define MTR_FRONT_LEFT_REV_IO       4
#define MTR_FRONT_LEFT_REV_TMR      LEDC_TIMER_2
#define MTR_FRONT_LEFT_REV          LEDC_CHANNEL_2
#define MTR_FRONT_LEFT_REV_DUTY     3361

#define MTR_FRONT_RIGHT_REV_IO      7
#define MTR_FRONT_RIGHT_REV_TMR     LEDC_TIMER_3
#define MTR_FRONT_RIGHT_REV         LEDC_CHANNEL_3
#define MTR_FRONT_RIGHT_REV_DUTY    3361

// Forward declaration
typedef struct motor_system_t motor_system_t;

// Struct definition
struct motor_system_t {
    int left_pwm;   // Signed PWM for left motors (-8191 to +8190)
    int right_pwm;  // Signed PWM for right motors (-8191 to +8190)

    // Internal motor PWM values
    int motor1_rpm_pcm;  // Left forward
    int motor2_rpm_pcm;  // Right forward
    int motor3_rpm_pcm;  // Left reverse
    int motor4_rpm_pcm;  // Right reverse

    void (*update)(motor_system_t *self, TickType_t now);
};

void motor_system_init(motor_system_t *sys);
void motor_set_pwm(motor_system_t *sys, int left_pwm, int right_pwm);
void update_motors_pwm(motor_system_t *sys, int pwm_motor_1, int pwm_motor_2);  // Your function

#endif
