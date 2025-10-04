/*  ESP32 Temperature Node

    BME280 I2C Slave Device

    Author:     Alexander Bobkov
    Date:       October 4, 2025
    Modified:   October 4, 2025
    
    
    Sensor device uses BME280 to measure temperature, pressure and humidity and 
    outputs it to the serial port and publishes values to the MQTT topics.
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

    bme280_set_sampling(bme280, BME280_MODE_NORMAL, BME280_SAMPLING_X1, BME280_SAMPLING_X1, BME280_SAMPLING_X1, BME280_FILTER_OFF, BME280_STANDBY_MS_1000); 

    while (true) {

        bme280_take_forced_measurement(bme280);
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        //printf("Temperature: %.1f C\n", temperature);
        //printf("Humidity: %.2f %%\n", humidity);
        //printf("Pressure: %.2f kPa\n", pressure);
        printf(" %.1f, %.2f, %.2f \n", temperature, humidity, pressure);

        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}