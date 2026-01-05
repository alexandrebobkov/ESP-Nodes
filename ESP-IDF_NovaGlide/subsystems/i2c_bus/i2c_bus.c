#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "I2C_BUS";

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

static i2c_master_bus_handle_t i2c_bus = NULL;

void i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus));
    ESP_LOGI(TAG, "I2C bus initialized");
}

i2c_master_bus_handle_t i2c_bus_get(void)
{
    return i2c_bus;
}
