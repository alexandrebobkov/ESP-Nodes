#include "ina219_sensor.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "INA219";

static void ina219_update_impl(ina219_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 2.5 seconds
    if ((now - last_read) >= pdMS_TO_TICKS(2500)) {
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&self->dev, &self->bus_voltage));
        ESP_ERROR_CHECK(ina219_get_shunt_voltage(&self->dev, &self->shunt_voltage));
        ESP_ERROR_CHECK(ina219_get_current(&self->dev, &self->current));
        ESP_ERROR_CHECK(ina219_get_power(&self->dev, &self->power));

        ESP_LOGI(TAG, "VBUS: %.2fV, I: %.2fmA, P: %.2fmW",
                 self->bus_voltage, self->current * 1000, self->power * 1000);

        last_read = now;
    }
}

void ina219_system_init(ina219_system_t *sys) {
    sys->bus_voltage = 0.0f;
    sys->shunt_voltage = 0.0f;
    sys->current = 0.0f;
    sys->power = 0.0f;

    memset(&sys->dev, 0, sizeof(ina219_t));
    sys->update = ina219_update_impl;

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina219_init_desc(&sys->dev, I2C_ADDR, I2C_PORT,
                                      I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(ina219_init(&sys->dev));
    ESP_ERROR_CHECK(ina219_configure(&sys->dev, INA219_BUS_RANGE_16V,
                                      INA219_GAIN_0_125,
                                      INA219_RES_12BIT_1S,
                                      INA219_RES_12BIT_1S,
                                      INA219_MODE_CONT_SHUNT_BUS));
    ESP_ERROR_CHECK(ina219_calibrate(&sys->dev,
                                      (float)SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    ESP_LOGI(TAG, "INA219 initialized");
}
