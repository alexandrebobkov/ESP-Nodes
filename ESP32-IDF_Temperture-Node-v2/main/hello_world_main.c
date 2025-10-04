
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
#include "esp_log.h"
#include "driver/i2c.h"
#include "bme280.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           21      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           22      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_ACKS                    0x1
#define I2C_ACKM                    0x0
#define I2C_NOACKM                  0x1                         // I2C NACK value
#define I2C_WRITE_BIT               I2C_MASTER_WRITE
#define I2C_READ_BIT                 I2C_MASTER_READ

#define MPU9250_SENSOR_ADDR                 0x76        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0xD0        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x88        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*static esp_err_t bme280_I2C_burst_read(uint8_t device_address, uint8_t register_address, uint8_t *registers_data, size_t count)
{
    int command_result = 0;
    //unsigned char array[BME280_I2C_BUFFER_LEN];
    unsigned char pos;
    array[0] = register_address;
    command_result = i2c_master_write_read_device(0, device_address, write_buffer, write_size, *array, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    //error = i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &register_address, 1, registers_data, count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}*/

static esp_err_t bme280_read_raw_temperature(uint8_t device_address, uint8_t *raw_temperature) {
    // BME280_INIT_VALUE LN 136
    // BME280_TEMPERATURE_MSB_REG 0xFA
    // BME280_TEMPERATURE_DATA_LENGTH
    // BME280_TEMPERATURE_DATA_SIZE
    // I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS

    esp_err_t command_result;
    //unsigned char raw_temperature[BME280_TEMPERATURE_DATA_SIZE] = {0, 0, 0};
    //command_result = i2c_master_read_from_device(0, BME280_I2C_ADDRESS1, *raw_temperature, BME280_TEMPERATURE_DATA_SIZE, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    command_result = i2c_master_write_read_device(0, device_address, *raw_temperature, BME280_TEMPERATURE_DATA_SIZE, BME280_TEMPERATURE_REGISTER, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return command_result;
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}*/

/**
 * @brief Initialize ESP32 as an I2C master device
 */
static esp_err_t i2c_driver_initialize(void)
{
    int i2c_master_port = 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags= 0,
    };

    return i2c_param_config(i2c_master_port, &conf);
}

void app_main(void)
{
    
}