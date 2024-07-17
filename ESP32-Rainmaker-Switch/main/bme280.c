#include "bme280.h"

static esp_err_t bme280_read_raw_temperature(uint8_t device_address, uint8_t *raw_temperature);
static esp_err_t bme280_get_id(uint8_t device_address);
