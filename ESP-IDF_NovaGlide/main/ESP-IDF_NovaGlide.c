#include "system_init.h"
#include "scheduler.h"
#include "control_task.h"

// Subsystems
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "wifi_sys.h"
#include "espnow_sys.h"
#include "mqtt_sys.h"
#include "ui.h"

// Telemetry bridge context
typedef struct {
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    motor_system_t *motors;
    mqtt_system_t *mqtt;
} telemetry_context_t;

// Task to bridge sensor data to MQTT
static void telemetry_bridge_task(void *arg) {
    telemetry_context_t *ctx = (telemetry_context_t *)arg;

    while (1) {
        // Update MQTT with latest sensor readings
        mqtt_update_temp(ctx->mqtt, ctx->temp->temperature);
        mqtt_update_battery(ctx->mqtt, ctx->ina->bus_voltage);
        mqtt_update_current(ctx->mqtt, ctx->ina->current * 1000.0f);
        mqtt_update_power(ctx->mqtt, ctx->ina->power);
        mqtt_update_pwm(ctx->mqtt, ctx->motors->left_pwm, ctx->motors->right_pwm);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    // System-level initialization
    system_init();

    // S
