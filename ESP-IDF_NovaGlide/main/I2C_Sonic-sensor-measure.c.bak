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

uint16_t measure_distance(uint8_t command)
{
    // Send command (0x50=cm, 0x51=inch, 0x52=us)
    esp_err_t ret = i2c_master_transmit(dev_handle, &command, 1, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Command 0x%02X send failed: %s", command, esp_err_to_name(ret));
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
    ESP_LOGI(TAG, "Cmd 0x%02X → [0x%02X 0x%02X 0x%02X 0x%02X] Distance: %u",
             command, data[0], data[1], data[2], data[3], distance);

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

    ESP_LOGI(TAG, "I2C initialized. Trying initialization sequences...\n");

    // Try potential init sequences
    ESP_LOGI(TAG, "Attempt 1: Sending reset/init command 0x00...");
    uint8_t init_cmds[] = {0x00, 0x01, 0x02, 0xFF};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_transmit(dev_handle, &init_cmds[i], 1, 500);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Try writing to config register
    ESP_LOGI(TAG, "Attempt 2: Writing to potential config registers...");
    uint8_t cfg1[] = {0x00, 0x01}; // Write 0x01 to reg 0x00
    i2c_master_transmit(dev_handle, cfg1, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg2[] = {0x01, 0x51}; // Write 0x51 to reg 0x01
    i2c_master_transmit(dev_handle, cfg2, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "\n>>> PLACE AN OBJECT 10-30cm IN FRONT OF SENSOR <<<\n");
    ESP_LOGI(TAG, "Starting continuous measurements...\n");

    vTaskDelay(pdMS_TO_TICKS(1000));

    int measurement_count = 0;

    while (1) {
        measurement_count++;

        ESP_LOGI(TAG, "─────────────────────────────────────────");
        ESP_LOGI(TAG, "Measurement #%d", measurement_count);

        // Try all three command types
        ESP_LOGI(TAG, "Testing 0x50 (cm):");
        uint16_t dist_cm = measure_distance(0x50);

        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_LOGI(TAG, "Testing 0x51 (inch):");
        uint16_t dist_in = measure_distance(0x51);

        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_LOGI(TAG, "Testing 0x52 (µs):");
        uint16_t dist_us = measure_distance(0x52);

        // Analysis
        ESP_LOGI(TAG, "\nResults: CM=%u, INCH=%u, µS=%u", dist_cm, dist_in, dist_us);

        if (dist_cm == 0 && dist_in == 0 && dist_us == 0) {
            ESP_LOGW(TAG, "⚠️  All zeros - sensor may be defective or not detecting");
        } else if (dist_cm > 0) {
            ESP_LOGI(TAG, "✅ Got valid reading: %u cm", dist_cm);
        }

        ESP_LOGI(TAG, "");

        // Wait 1 second between measurement cycles
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
