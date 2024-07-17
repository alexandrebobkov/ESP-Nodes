#ifndef __BME280_H__
#define __BME280_H__

#include "driver/i2c.h"
#include "esp_log.h"

#define BME280_CHIP_ID                      (0xD0)
#define BME280_I2C_ADDRESS1                 (0x76)      // BME280 Data sheet p. 32
#define BME280_I2C_ADDRESS2                 (0x77)
#define BME280_TEMPERATURE_REGISTER         (0xFA)

#define BME280_TEMPERATURE_DATA_SIZE        (3)
#define BME280_TEMPERATURE_DATA_LENGTH      (3)
#define BME280_PRESSURE_DATA_SIZE           (3)
#define BME280_PRESSURE_DATA_LENGTH         (3)
#define BME280_HUMIDITY_DATA_SIZE           (2)
#define BME280_HUMIDITY_DATA_LENGTH         (2)

#define I2C_MASTER_TX_BUF_DISABLE           (0)                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE           (0)                          /*!< I2C master doesn't need buffer */



/** GLOBAL MEMORY MAN AND REGISTERS
 * BME280 Data Sheet p. 26
 * 0xD0         -> Chip ID      -> 0x60 for BME280 or 0x56;0x58 for BMP280
 * 0xF3         -> Status       -> 
 * 0xE0         -> Reset        -> Write 0xB6 for complete power-on-reset procedure (BME280 Data Sheet p. 27)
 * 0xF5         -> Config       -> 110 for 10ms or 111 for 20ms
 * 0xF7-0xF9    -> Pressure     -> 16- to 20-bit resolution, unsigned
 * 0xFA-0xFC    -> Temperature  -> 16- to 20-bit resolution, unsigned
 * 0xFD-0xFE    -> Humidity     -> 16-bit resolution
 */

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} sensors;

void init_bme280();

static esp_err_t bme280_read_raw_temperature(uint8_t device_address, uint8_t *raw_temperature);
static esp_err_t bme280_get_id(uint8_t device_address);

#endif


