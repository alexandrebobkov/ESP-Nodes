#include "ultrasonic_sensor.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

#define ULTRASONIC_I2C_ADDR    0x57
#define CMD_MEASURE_CM         0x50

static i2c_master_dev_handle_t ultrasonic_handle = NULL;

// EXACT code from working test - measure_distance function
static uint16_t measure_distance(uint8_t command)
{
    // Send command (0x50=cm, 0x51=inch, 0x52=us)
    esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &command, 1, 500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Command 0x%02X send failed: %s", command, esp_err_to_name(ret));
        return 0xFFFF;
    }

    // Wait for measurement
    vTaskDelay(pdMS_TO_TICKS(70));

    // Read 4 bytes
    uint8_t data[4];
    ret = i2c_master_receive(ultrasonic_handle, data, 4, 500);
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

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 500ms (slower to match working test)
    if ((now - last_read) < pdMS_TO_TICKS(500)) {
        return;
    }
    last_read = now;

    // Measure using 0x50 (cm)
    uint16_t dist_cm = measure_distance(0x50);

    if (dist_cm == 0xFFFF) {
        self->measurement_valid = false;
        ESP_LOGE(TAG, "❌ Communication error!");
    } else if (dist_cm == 0) {
        self->measurement_valid = false;
        self->distance_cm = 0.0f;
        ESP_LOGW(TAG, "⚠️  No object detected");
    } else if (dist_cm > 0 && dist_cm <= 400) {
        self->measurement_valid = true;
        self->distance_cm = (float)dist_cm;
        ESP_LOGI(TAG, "✅ Valid distance: %u cm", dist_cm);
    } else {
        self->measurement_valid = false;
        self->distance_cm = (float)dist_cm;
        ESP_LOGW(TAG, "⚠️  Out of range: %u cm", dist_cm);
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->measurement_valid = false;
    sys->update = ultrasonic_update_impl;

    // Add device to I2C bus
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "ULTRASONIC", &ultrasonic_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Ultrasonic initialized at 0x%02X", ULTRASONIC_I2C_ADDR);

    // EXACT init sequence from working test
    ESP_LOGI(TAG, "Sending init commands...");
    uint8_t init_cmds[] = {0x00, 0x01, 0x02, 0xFF};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 500);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Config register writes from working test
    uint8_t cfg1[] = {0x00, 0x01};
    i2c_master_transmit(ultrasonic_handle, cfg1, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg2[] = {0x01, 0x51};
    i2c_master_transmit(ultrasonic_handle, cfg2, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init complete, ready to measure");
}
