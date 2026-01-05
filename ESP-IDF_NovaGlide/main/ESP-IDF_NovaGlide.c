#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC_TEST";

#define I2C_MASTER_SCL_IO      2
#define I2C_MASTER_SDA_IO      3
#define I2C_MASTER_FREQ_HZ     100000
#define DEVICE_ADDRESS         0x57
#define CMD_MEASURE_CM         0x50

i2c_master_dev_handle_t dev_handle;

uint16_t measure_distance(void)
{
    // Send 0x50 command
    uint8_t cmd = CMD_MEASURE_CM;
    esp_err_t ret = i2c_master_transmit(dev_handle, &cmd, 1, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Command send failed: %s", esp_err_to_name(ret));
        return 0xFFFF;
    }

    // Wait for measurement
    vTaskDelay(pdMS_TO_TICKS(70));

    // Read 4 bytes
    uint8_t data[4];
    ret = i2c_master_receive(dev_handle, data, 4, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        return 0xFFFF;
    }

    // Extract distance
    uint16_t distance = (data[0] << 8) | data[1];

    // Log all bytes for debugging
    ESP_LOGI(TAG, "Raw: [0x%02X 0x%02X 0x%02X 0x%02X] → Distance: %u cm",
             data[0], data[1], data[2], data[3], distance);

    return distance;
}

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, "  Ultrasonic Sensor Live Test");
    ESP_LOGI(TAG, "  Address: 0x%02X | Command: 0x%02X", DEVICE_ADDRESS, CMD_MEASURE_CM);
    ESP_LOGI(TAG, "═══════════════════════════════════════════\n");

    // Initialize I2C
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEVICE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_LOGI(TAG, "I2C initialized. Starting continuous measurements...\n");
    ESP_LOGI(TAG, ">>> PLACE AN OBJECT 10-30cm IN FRONT OF SENSOR <<<\n");

    vTaskDelay(pdMS_TO_TICKS(1000));

    int measurement_count = 0;

    while (1) {
        measurement_count++;

        ESP_LOGI(TAG, "─────────────────────────────────────────");
        ESP_LOGI(TAG, "Measurement #%d", measurement_count);

        uint16_t distance = measure_distance();

        if (distance == 0xFFFF) {
            ESP_LOGE(TAG, "❌ Communication error!");
        } else if (distance == 0) {
            ESP_LOGW(TAG, "⚠️  No object detected (0 cm)");
        } else if (distance < 2) {
            ESP_LOGW(TAG, "⚠️  Object too close: %u cm", distance);
        } else if (distance > 400) {
            ESP_LOGW(TAG, "⚠️  Object too far: %u cm", distance);
        } else {
            ESP_LOGI(TAG, "✅ Valid distance: %u cm", distance);
        }

        // Wait 500ms between measurements
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
