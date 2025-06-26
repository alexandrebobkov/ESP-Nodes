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