#ifndef INA219_SENSOR_H
#define INA219_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "ina219.h"

//#define I2C_PORT 0
#define I2C_ADDR 0x40
#define I2C_SDA_GPIO 3
#define I2C_SCL_GPIO 2
#define SHUNT_RESISTOR_MILLI_OHM 100

// Forward declaration
typedef struct ina219_system_t ina219_system_t;

// Struct definition
struct ina219_system_t {
    float bus_voltage;
    float shunt_voltage;
    float current;
    float power;

    ina219_t dev;

    void (*update)(ina219_system_t *self, TickType_t now);
};

void ina219_system_init(ina219_system_t *sys);

#endif
