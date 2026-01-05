#include "ultrasonic_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

// Distance register discovered by probing
#define ULTRA_REG_DISTANCE 0x02

// Read 16-bit register (INA219 style)
static esp_err_t ultrasonic_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    esp_err_t ret = i2c_bus_read(dev, reg, data, 2);

    if (ret == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }

    return ret;
}

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now)
{
    static TickType_t last = 0;

    if ((now - last) < pdMS_TO_TICKS(200)) {
        return;
    }
    last = now;

    uint16_t raw_mm = 0;
    esp_err_t ret = ultrasonic_read_reg(self->dev, ULTRA_REG_DISTANCE, &raw_mm);

    if (ret == ESP_OK) {
        self->distance_cm = raw_mm / 10.0f;
        ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
    } else {
        ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys)
{
    sys->distance_cm = 0.0f;
    sys->update = ultrasonic_update_impl;

    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "HC-SR04_I2C", &sys->dev);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic sensor");
        return;
    }

    ESP_LOGI(TAG, "Ultrasonic I2C sensor initialized at 0x%02X", ULTRASONIC_I2C_ADDR);
}
