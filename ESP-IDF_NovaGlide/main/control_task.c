#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static joystick_hal_t js;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_left = 0;
    int pwm_right = 0;

    ESP_LOGI(TAG, "Control task started");

    // Initialize joystick HAL
    joystick_hal_init(&js);

    while (1) {
        // 1. Read raw joystick values from ESP-NOW
        int32_t rc_x = ctx->espnow->last_data.x_axis;
        int32_t rc_y = ctx->espnow->last_data.y_axis;

        // 2. Update joystick HAL (auto-calibration + normalization)
        js.update(&js, rc_x, rc_y);

        // 3. Mix normalized joystick values into motor PWM
        joystick_mix(js.norm_x, js.norm_y, &pwm_left, &pwm_right);

        // 4. Apply PWM to motors
        update_motors_pwm(ctx->motors, pwm_left, pwm_right);

        // 5. Debug output
        ESP_LOGI(TAG,
                 "RC raw=(%ld,%ld) norm=(%.2f,%.2f) PWM(L,R)=(%d,%d)",
                 (long)rc_x, (long)rc_y,
                 js.norm_x, js.norm_y,
                 pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz control loop
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 4096, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}
