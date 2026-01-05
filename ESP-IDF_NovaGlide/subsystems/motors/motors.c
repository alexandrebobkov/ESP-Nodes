#include "motors.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "MOTORS";

// Initialize LEDC timers and channels
static void ledc_init(void) {
    // Motor 1 - Left Forward
    ledc_timer_config_t timer1 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_LEFT_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer1));

    ledc_channel_config_t channel1 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_LEFT,
        .timer_sel = MTR_FRONT_LEFT_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_LEFT_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel1));

    // Motor 2 - Right Forward
    ledc_timer_config_t timer2 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_RIGHT_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer2));

    ledc_channel_config_t channel2 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_RIGHT,
        .timer_sel = MTR_FRONT_RIGHT_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_RIGHT_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel2));

    // Motor 3 - Left Reverse
    ledc_timer_config_t timer3 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_LEFT_REV_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer3));

    ledc_channel_config_t channel3 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_LEFT_REV,
        .timer_sel = MTR_FRONT_LEFT_REV_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_LEFT_REV_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel3));

    // Motor 4 - Right Reverse
    ledc_timer_config_t timer4 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_RIGHT_REV_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer4));

    ledc_channel_config_t channel4 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_RIGHT_REV,
        .timer_sel = MTR_FRONT_RIGHT_REV_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_RIGHT_REV_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel4));

    ESP_LOGI(TAG, "LEDC initialized for all 4 motors");
}

/*
void update_motors_pwm(motor_system_t *sys, int pwm_left, int pwm_right)
{
    // LEFT MOTOR
    if (pwm_left >= 0) {
        sys->motor1_rpm_pcm = pwm_left;     // L-FWD
        sys->motor3_rpm_pcm = 0;            // L-REV
    } else {
        sys->motor1_rpm_pcm = 0;
        sys->motor3_rpm_pcm = -pwm_left;
    }

    // RIGHT MOTOR
    if (pwm_right >= 0) {
        sys->motor2_rpm_pcm = pwm_right;    // R-FWD
        sys->motor4_rpm_pcm = 0;            // R-REV
    } else {
        sys->motor2_rpm_pcm = 0;
        sys->motor4_rpm_pcm = -pwm_right;
    }

    // Store signed values
    sys->left_pwm = pwm_left;
    sys->right_pwm = pwm_right;

    // Update hardware
    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT,      sys->motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT,     sys->motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV,  sys->motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, sys->motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    ESP_LOGI(TAG, "M1(L-FWD): %d, M2(R-FWD): %d, M3(L-REV): %d, M4(R-REV): %d",
             sys->motor1_rpm_pcm,
             sys->motor2_rpm_pcm,
             sys->motor3_rpm_pcm,
             sys->motor4_rpm_pcm);
}
*/


// Your proven motor update logic
void update_motors_pwm(motor_system_t *sys, int pwm_motor_1, int pwm_motor_2) {
    /* UPDATED MOTOR LOGIC */
    if (pwm_motor_1 >= 0 && pwm_motor_2 >= 0) {
        sys->motor1_rpm_pcm = pwm_motor_1;
        sys->motor2_rpm_pcm = pwm_motor_2;
        sys->motor3_rpm_pcm = 0;
        sys->motor4_rpm_pcm = 0;
    }
    if (pwm_motor_1 > 0 && pwm_motor_2 < 0) {
        sys->motor1_rpm_pcm = 0;
        sys->motor2_rpm_pcm = pwm_motor_1;
        sys->motor3_rpm_pcm = 0;
        sys->motor4_rpm_pcm = -pwm_motor_2;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 > 0) {
        sys->motor1_rpm_pcm = -pwm_motor_1;
        sys->motor2_rpm_pcm = 0;
        sys->motor3_rpm_pcm = pwm_motor_2;
        sys->motor4_rpm_pcm = 0;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 < 0) {
        sys->motor1_rpm_pcm = 0;
        sys->motor2_rpm_pcm = 0;
        sys->motor3_rpm_pcm = -pwm_motor_1;
        sys->motor4_rpm_pcm = -pwm_motor_2;
    }

    // Store the input PWM values
    sys->left_pwm = pwm_motor_1;
    sys->right_pwm = pwm_motor_2;

    // Update hardware
    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, sys->motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, sys->motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, sys->motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, sys->motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    ESP_LOGI(TAG, "M1: %d, M2: %d, M3: %d, M4: %d",
        sys->motor1_rpm_pcm,
        sys->motor2_rpm_pcm,
        sys->motor3_rpm_pcm,
        sys->motor4_rpm_pcm);
}

// Update motor PWM values (unused in your case, but kept for compatibility)
static void motor_update_impl(motor_system_t *self, TickType_t now) {
    (void)self;
    (void)now;
    // Nothing to do - motors are updated directly via update_motors_pwm()
}

void motor_system_init(motor_system_t *sys) {
    sys->left_pwm = 0;
    sys->right_pwm = 0;
    sys->motor1_rpm_pcm = 0;
    sys->motor2_rpm_pcm = 0;
    sys->motor3_rpm_pcm = 0;
    sys->motor4_rpm_pcm = 0;
    sys->update = motor_update_impl;

    ledc_init();
    ESP_LOGI(TAG, "Motor system initialized");
}

void motor_set_pwm(motor_system_t *sys, int left_pwm, int right_pwm) {
    // This is just a wrapper for update_motors_pwm
    update_motors_pwm(sys, left_pwm, right_pwm);
}
