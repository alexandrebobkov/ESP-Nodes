#ifndef SENSORS_DATA_H
#define SENSORS_DATA_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t    crc;                
    int         sensor1;            
    int         sensor2;             
    bool        sensor3;          
    bool        sensor4;              
    uint8_t     motor1_rpm_pwm;     
    uint8_t     motor2_rpm_pwm;
    uint8_t     motor3_rpm_pwm;
    uint8_t     motor4_rpm_pwm;
} __attribute__((packed)) sensors_data_t;

#endif