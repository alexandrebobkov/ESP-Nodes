#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "I2C_BRUTE";

#define I2C_MASTER_SCL_IO           2
#define I2C_MASTER_SDA_IO           3
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       200    // Shorter timeout for speed

#define DEVICE_ADDRESS              0x57

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

// Test all possible single-byte commands (0x00-0xFF)
void brute_force_commands(void)
{
    ESP_LOGI(TAG, "\n╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BRUTE FORCE: Testing all command bytes 0x00-0xFF ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝\n");

    int successful_commands = 0;

    for (uint16_t cmd_byte = 0x00; cmd_byte <= 0xFF; cmd_byte++) {
        uint8_t cmd = (uint8_t)cmd_byte;

        // Send command
        esp_err_t ret = i2c_master_transmit(dev_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
        if (ret != ESP_OK) {
            continue; // Skip if can't send
        }

        // Wait for potential response
        vTaskDelay(pdMS_TO_TICKS(50));

        // Try to read response
        uint8_t data[8];
        ret = i2c_master_receive(dev_handle, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);

        if (ret == ESP_OK) {
            // Check if data is not all zeros or all 0xFF (likely real data)
            bool has_data = false;
            for (int i = 0; i < 4; i++) {
                if (data[i] != 0x00 && data[i] != 0xFF) {
                    has_data = true;
                    break;
                }
            }

            if (has_data || (data[0] == 0x00 && data[1] != 0x00)) {
                ESP_LOGI(TAG, "✓ CMD 0x%02X → 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         cmd, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                successful_commands++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between commands
    }

    ESP_LOGI(TAG, "\n>>> Found %d commands with responses\n", successful_commands);
}

// Test register-based access (write register address, then read)
void brute_force_registers(void)
{
    ESP_LOGI(TAG, "\n╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BRUTE FORCE: Testing register reads 0x00-0xFF    ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝\n");

    int readable_regs = 0;

    for (uint16_t reg = 0x00; reg <= 0xFF; reg++) {
        uint8_t reg_addr = (uint8_t)reg;
        uint8_t data[4];

        esp_err_t ret = i2c_master_transmit_receive(
            dev_handle,
            &reg_addr, 1,
            data, sizeof(data),
            I2C_MASTER_TIMEOUT_MS
        );

        if (ret == ESP_OK) {
            // Check for meaningful data
            bool has_data = false;
            for (int i = 0; i < sizeof(data); i++) {
                if (data[i] != 0x00 && data[i] != 0xFF) {
                    has_data = true;
                    break;
                }
            }

            if (has_data) {
                ESP_LOGI(TAG, "✓ REG 0x%02X = 0x%02X 0x%02X 0x%02X 0x%02X",
                         reg, data[0], data[1], data[2], data[3]);
                readable_regs++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "\n>>> Found %d readable registers\n", readable_regs);
}

// Test 2-byte command sequences (cmd + parameter)
void brute_force_two_byte_commands(void)
{
    ESP_LOGI(TAG, "\n╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BRUTE FORCE: Testing 2-byte command patterns     ║");
    ESP_LOGI(TAG, "║  (This will take a while - testing key combos)    ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝\n");

    // Test common register write patterns: [reg_addr, value]
    uint8_t test_regs[] = {0x00, 0x01, 0x02, 0x03, 0x10, 0x20, 0x30, 0xF0, 0xFE, 0xFF};
    uint8_t test_vals[] = {0x00, 0x01, 0x02, 0x50, 0x51, 0x52, 0x55, 0xAA, 0xFF};

    int found = 0;

    for (int r = 0; r < sizeof(test_regs); r++) {
        for (int v = 0; v < sizeof(test_vals); v++) {
            uint8_t cmd[2] = {test_regs[r], test_vals[v]};

            esp_err_t ret = i2c_master_transmit(dev_handle, cmd, 2, I2C_MASTER_TIMEOUT_MS);
            if (ret != ESP_OK) continue;

            vTaskDelay(pdMS_TO_TICKS(50));

            uint8_t data[4];
            ret = i2c_master_receive(dev_handle, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);

            if (ret == ESP_OK) {
                bool has_data = false;
                for (int i = 0; i < sizeof(data); i++) {
                    if (data[i] != 0x00 && data[i] != 0xFF) {
                        has_data = true;
                        break;
                    }
                }

                if (has_data) {
                    ESP_LOGI(TAG, "✓ [0x%02X, 0x%02X] → 0x%02X 0x%02X 0x%02X 0x%02X",
                             cmd[0], cmd[1], data[0], data[1], data[2], data[3]);
                    found++;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "\n>>> Found %d working 2-byte sequences\n", found);
}

// Test if device responds to specific probe patterns
void test_identification(void)
{
    ESP_LOGI(TAG, "\n╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Testing device identification patterns           ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝\n");

    // Common ID/version register addresses
    uint8_t id_regs[] = {0x00, 0x01, 0xFE, 0xFF, 0x0F, 0xA0, 0xD0};

    for (int i = 0; i < sizeof(id_regs); i++) {
        uint8_t reg = id_regs[i];
        uint8_t data[8];

        esp_err_t ret = i2c_master_transmit_receive(
            dev_handle,
            &reg, 1,
            data, sizeof(data),
            I2C_MASTER_TIMEOUT_MS
        );

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ID Reg 0x%02X: %02X %02X %02X %02X %02X %02X %02X %02X",
                     reg, data[0], data[1], data[2], data[3],
                     data[4], data[5], data[6], data[7]);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  I2C Protocol Brute Force Scanner");
    ESP_LOGI(TAG, "  Device: 0x%02X | SDA: GPIO%d | SCL: GPIO%d",
             DEVICE_ADDRESS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════\n");

    // Initialize I2C
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEVICE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    vTaskDelay(pdMS_TO_TICKS(500));

    // Run all discovery tests
    test_identification();
    vTaskDelay(pdMS_TO_TICKS(500));

    brute_force_commands();
    vTaskDelay(pdMS_TO_TICKS(500));

    brute_force_registers();
    vTaskDelay(pdMS_TO_TICKS(500));

    brute_force_two_byte_commands();

    ESP_LOGI(TAG, "\n\n═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  SCAN COMPLETE!");
    ESP_LOGI(TAG, "  Check the '✓' lines above for working protocols");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════\n");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
