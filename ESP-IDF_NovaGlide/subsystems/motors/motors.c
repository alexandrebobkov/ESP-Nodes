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

// Update motor PWM values
static void motor_update_impl(motor_system_t *self, TickType_t now) {
    (void)now;

    // Apply left motor PWM (signed)
    if (self->left_pwm >= 0) {
        self->motor1_rpm_pcm = self->left_pwm;
        self->motor3_rpm_pcm = 0;
    } else {
        self->motor1_rpm_pcm = 0;
        self->motor3_rpm_pcm = -self->left_pwm;
    }

    // Apply right motor PWM (signed)
    if (self->right_pwm >= 0) {
        self->motor2_rpm_pcm = self->right_pwm;
        self->motor4_rpm_pcm = 0;
    } else {
        self->motor2_rpm_pcm = 0;
        self->motor4_rpm_pcm = -self->right_pwm;
    }

    // Update hardware
    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, self->motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, self->motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, self->motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, self->motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    // Log every second
    static TickType_t last_log = 0;
    if ((now - last_log) >= pdMS_TO_TICKS(1000)) {
        ESP_LOGI(TAG, "PWM L/R: %d/%d | M1:%d M2:%d M3:%d M4:%d",
                 self->left_pwm, self->right_pwm,
                 self->motor1_rpm_pcm, self->motor2_rpm_pcm,
                 self->motor3_rpm_pcm, self->motor4_rpm_pcm);
        last_log = now;
    }
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
    // Clamp values
    if (left_pwm > 8190) left_pwm = 8190;
    if (left_pwm < -8191) left_pwm = -8191;
    if (right_pwm > 8190) right_pwm = 8190;
    if (right_pwm < -8191) right_pwm = -8191;

    sys->left_pwm = left_pwm;
    sys->right_pwm = right_pwm;
}
