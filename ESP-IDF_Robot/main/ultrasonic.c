#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_timer.h>

#include "ultrasonic.h"

esp_err_t ultrasonic_init (const ultrasonic_sensor_t *sensor)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure the trigger GPIO
    gpio_config_t trigger_config = {
        .pin_bit_mask = (1ULL << sensor->trigger_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&trigger_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure the echo GPIO
    gpio_config_t echo_config = {
        .pin_bit_mask = (1ULL << sensor->echo_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&echo_config);
    
    return ret;
}

esp_err_t ultrasonic_measure_raw (const ultrasonic_sensor_t *sensor, uint32_t max_time_us, uint32_t *time_us)
{
    if (sensor == NULL || time_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send a 10us pulse to the trigger pin
    gpio_set_level(sensor->trigger_gpio, 1);
    vTaskDelay (10 / portTICK_PERIOD_MS); 
    gpio_set_level(sensor->trigger_gpio, 0);

    // Wait for the echo pin to go high
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_gpio) == 0) {
        if (esp_timer_get_time() - start_time > max_time_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    // Measure the duration of the high signal on the echo pin
    start_time = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_gpio) == 1) {
        if (esp_timer_get_time() - start_time > max_time_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    *time_us = esp_timer_get_time() - start_time;

    return ESP_OK;
}