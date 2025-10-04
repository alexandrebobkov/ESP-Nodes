
/*  ESP32 Module I2C Master Device

    Author:     Alexander Bobkov
    Date:       July 3, 2024
    Modified:   July 9, 2024
    
    
    Adopted from i2c - Simple example

    Simple I2C example that shows how to initialize I2C
    as well as reading and writing from and to registers for a sensor connected over I2C.

    The sensor used in this example is a MPU9250 inertial measurement unit.

    For other examples please check:
    https://github.com/espressif/esp-idf/tree/master/examples

    See README.md file to get detailed usage of this example.

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "bme280.h"

static const char *TAG = "i2c-simple-example";
float temperature = 0.0f, humidity = 0.0f, pressure = 0.0f;

int i2c_master_port = 0;
static i2c_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280 = NULL;

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 21,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = 22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
    .clk_flags= 0,
};


void app_main(void)
{
    i2c_bus = i2c_bus_create(i2c_master_port, &conf);
    bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    bme280_default_init(bme280);

    bme280_set_sampling(bme280, BME28_0SAMPLING_X1); 

    while (true) {

        bme280_take_forced_measurement(bme280);
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        printf("Temperature: %.2f C\n", temperature);
        printf("Temperature: %.2f \n", humidity);
        printf("Temperature: %.2f kPa\n", pressure);

        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

/*
#include <stdio.h>
#include "esp_log.h"

void app_main(void)
{
    
}
*/