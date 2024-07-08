#ifndef __BME280_H__
#define __BME280_H__

#define BME280_CHIP_ID          (0x60)
#define BME280_I2C_ADDRESS1     (0x76)      // BME280 Data sheet p. 32
#define BME280_I2C_ADDRESS2     (0x77)

/** GLOBAL MEMORY MAN AND REGISTERS
 * BME280 Data Sheet p. 26
 * 0xD0         -> Chip ID      -> 0x60 for BME280 or 0x56;0x58 for BMP280
 * 0xF5         -> Config       -> 110 for 10ms or 111 for 20ms
 * 0xF7-0xF9    -> Pressure     -> 16- to 20-bit resolution
 * 0xFA-0xFC    -> Temperature  -> 16- to 20-bit resolution
 */


