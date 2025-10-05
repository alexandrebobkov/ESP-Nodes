/*  ESP32 Temperature Node

    BME280 I2C Slave Device

    Author:     Alexander Bobkov
    Date:       October 4, 2025
    Modified:   October 4, 2025
    
    Sensor device uses BME280 to measure temperature, pressure and humidity and 
    outputs it to the serial port and publishes values to the MQTT topics.

    Built and Compiled with ESP-IDF v5.4.1
    Uses Espressif BME280 component: espressif/bme280: ^0.1.1
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "config.h"

static const char *TAG = "esp32 node";

// The three variables for storing the sensor readings (float for decimals)
float temperature = 0.0f, humidity = 0.0f, pressure = 0.0f;

int i2c_master_port = 0;
// I2C bus handle
static i2c_bus_handle_t i2c_bus = NULL;
// BME280 device handle
static bme280_handle_t bme280 = NULL;

// I2C bus configuration struct
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_FREQ_HZ,
    .clk_flags= 0,
};

// Task to read sensor values
void read_sensor_task(void *arg)
{
    while (1) {
        // Take a forced measurement
        bme280_take_forced_measurement(bme280);
        // Read temperature, humidity, and pressure values
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

// Task to print sensor values
void print_sensor_task(void *arg)
{
    while (1) {
        // Print the values to the serial console
        printf(" %.1f, %.2f, %.2f \n", temperature, humidity, pressure);
        vTaskDelay(2500/portTICK_PERIOD_MS);
    }
}   

void app_main(void)
{
    // Create the I2C bus and BME280 device handles
    i2c_bus = i2c_bus_create(i2c_master_port, &conf);
    bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    // Initialize the BME280 device
    bme280_default_init(bme280);

    // Set the BME280 sampling parameters
    bme280_set_sampling(bme280, BME280_MODE_NORMAL, BME280_SAMPLING_X1, BME280_SAMPLING_X1, BME280_SAMPLING_X1, BME280_FILTER_OFF, BME280_STANDBY_MS_1000); 

    // Create FreeRTOS tasks for reading and printing sensor values
    xTaskCreate(read_sensor_task, "read_sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(print_sensor_task, "print_sensor_task", 2048, NULL, 5, NULL);

    // Main loop to read and print the sensor values every 2 seconds
    /*while (true) {

        bme280_take_forced_measurement(bme280);
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        // Print the values to the serial console
        printf(" %.1f, %.2f, %.2f \n", temperature, humidity, pressure);

        vTaskDelay(2000/portTICK_PERIOD_MS);
    }*/
}