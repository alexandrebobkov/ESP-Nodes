#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "I2C_REG_SCANNER";

// I2C Configuration
#define I2C_MASTER_SCL_IO           2      // GPIO for SCL
#define I2C_MASTER_SDA_IO           3      // GPIO for SDA
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz
#define I2C_MASTER_TIMEOUT_MS       1000

#define DEVICE_ADDRESS              0x57   // Your ultrasonic sensor address

void scan_registers(i2c_master_dev_handle_t dev_handle)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Scanning registers at device 0x%02X                    ║", DEVICE_ADDRESS);
    ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");

    int readable_regs = 0;

    // Method 1: Try reading each register (0x00 to 0xFF)
    ESP_LOGI(TAG, "║  Method 1: Single register reads                       ║");
    for (uint16_t reg = 0x00; reg <= 0xFF; reg++) {
        uint8_t reg_addr = (uint8_t)reg;
        uint8_t data;

        esp_err_t ret = i2c_master_transmit_receive(
            dev_handle,
            &reg_addr, 1,      // Write register address
            &data, 1,          // Read 1 byte
            I2C_MASTER_TIMEOUT_MS
        );

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "║  Reg 0x%02X = 0x%02X                                    ║", reg, data);
            readable_regs++;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between reads
    }

    ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║  Readable registers found: %d                          ║", readable_regs);
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Method 2: Try direct read (no register address, some sensors work this way)
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Method 2: Direct read (no register address)           ║");
    ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");

    uint8_t direct_data[8];
    esp_err_t ret = i2c_master_receive(dev_handle, direct_data, sizeof(direct_data), I2C_MASTER_TIMEOUT_MS);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "║  Direct read SUCCESS! Data:                            ║");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, direct_data, sizeof(direct_data), ESP_LOG_INFO);
    } else {
        ESP_LOGI(TAG, "║  Direct read failed: %s                     ║", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Method 3: Try sending measurement command then reading
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Method 3: Common ultrasonic sensor commands           ║");
    ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");

    // Common command bytes for ultrasonic sensors
    uint8_t commands[] = {0x50, 0x51, 0x52, 0x01, 0x02, 0x55};
    const char *cmd_names[] = {
        "0x50 (Range cm)",
        "0x51 (Range inch)",
        "0x52 (Range µs)",
        "0x01 (Trigger)",
        "0x02 (Read)",
        "0x55 (Start)"
    };

    for (int i = 0; i < sizeof(commands); i++) {
        ESP_LOGI(TAG, "║  Trying command: %s", cmd_names[i]);

        // Send command
        ret = i2c_master_transmit(dev_handle, &commands[i], 1, I2C_MASTER_TIMEOUT_MS);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "║    Command failed to send                              ║");
            continue;
        }

        // Wait for measurement (typical 30-100ms)
        vTaskDelay(pdMS_TO_TICKS(70));

        // Try to read result
        uint8_t result[4];
        ret = i2c_master_receive(dev_handle, result, sizeof(result), I2C_MASTER_TIMEOUT_MS);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "║    SUCCESS! Data: 0x%02X 0x%02X 0x%02X 0x%02X              ║",
                     result[0], result[1], result[2], result[3]);

            // Try interpreting as 16-bit distance
            uint16_t distance = (result[0] << 8) | result[1];
            ESP_LOGI(TAG, "║    As 16-bit value: %u                                 ║", distance);
        } else {
            ESP_LOGI(TAG, "║    Read failed: %s                          ║", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Register Scanner for device 0x%02X...", DEVICE_ADDRESS);

    // Initialize I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // Add device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEVICE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_LOGI(TAG, "I2C initialized, starting scan...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run scan once
    scan_registers(dev_handle);

    ESP_LOGI(TAG, "\nScan complete! Check the logs above for working commands.");
    ESP_LOGI(TAG, "Common patterns:");
    ESP_LOGI(TAG, "  - If Method 1 found registers: Use register-based access");
    ESP_LOGI(TAG, "  - If Method 2 worked: Use direct reads");
    ESP_LOGI(TAG, "  - If Method 3 worked: Note which command succeeded");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
