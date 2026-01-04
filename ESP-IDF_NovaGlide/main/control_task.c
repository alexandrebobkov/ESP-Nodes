#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_motor_1, pwm_motor_2;

    ESP_LOGI(TAG, "Control task started");

    while (1) {
        // Get joystick values from ESP-NOW (rc_x, rc_y)
        int rc_x = ctx->espnow->last_data.x_axis;
        int rc_y = ctx->espnow->last_data.y_axis;

        // Apply joystick mixing algorithm
        joystick_mix(rc_y, rc_x, &pwm_motor_1, &pwm_motor_2);

        // Update motors using your proven function
        update_motors_pwm(ctx->motors, pwm_motor_1, pwm_motor_2);

        ESP_LOGI(TAG, "RC(x,y): (%d,%d) -> PWM(L,R): (%d,%d)",
                 rc_x, rc_y, pwm_motor_1, pwm_motor_2);

        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz, matches your original
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 2048, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}
