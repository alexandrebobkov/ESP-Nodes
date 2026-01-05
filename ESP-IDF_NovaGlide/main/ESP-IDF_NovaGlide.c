#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "system_init.h"
#include "wifi_sys.h"
#include "i2c_bus.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"

static const char *TAG = "ULTRA_TEST_APP";

void app_main(void)
{
    // Basic system init (clocks, logging, etc.)
    system_init();

    // Optional: WiFi (you can comment this out if you want it quieter)
    wifi_system_init();

    // Subsystem instances (only what we actually use here)
    static ina219_system_t ina;
    static ultrasonic_system_t ultra;

    // --- I2C BUS INIT ---
    ESP_LOGI(TAG, "Initializing I2C bus...");
    ESP_ERROR_CHECK(i2c_bus_init());

    ESP_LOGI(TAG, "Scanning I2C bus...");
    i2c_bus_scan();   // should show 0x40 (INA219) and 0x57 (ultrasonic)

    // --- I2C DEVICES INIT ---
    ESP_LOGI(TAG, "Initializing INA219...");
    ina219_system_init(&ina);

    ESP_LOGI(TAG, "Initializing ultrasonic (HC-SR04 I2C)...");
    ultrasonic_system_init(&ultra);

    // Small delay to let the ultrasonic module settle after init
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Starting raw ultrasonic read loop...");

    while (1) {
        uint8_t data[2] = {0};
        esp_err_t err = i2c_master_receive(ultra.dev, data, 2, pdMS_TO_TICKS(200));

        if (err == ESP_OK) {
            uint16_t raw_mm = ((uint16_t)data[0] << 8) | data[1];
            float cm = raw_mm / 10.0f;
            ESP_LOGI("ULTRA_RAW", "raw=%u mm, %.2f cm", raw_mm, cm);
        } else {
            ESP_LOGW("ULTRA_RAW", "raw read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
