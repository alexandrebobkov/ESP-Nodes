# Project Source Archive

Generated automatically for analysis.

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/CMakeLists.txt`

```text
idf_component_register(SRCS "mqtt.c" "i2cdev.c" "ina219.c" "ultrasonic.c" "blink_example_main.c"
                       INCLUDE_DIRS ".")

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/ina219.h`

```text
/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ina219.h
 * @defgroup ina219 ina219
 * @{
 *
 * ESP-IDF driver for INA219/INA220 Zerø-Drift, Bidirectional
 * Current/Power Monitor
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __INA219_H__
#define __INA219_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA219_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA219_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA219_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA219_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA219_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA219_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA219_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA219_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA219_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA219_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA219_ADDR_SDA_SDA 0x4a //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA219_ADDR_SDA_SCL 0x4b //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA219_ADDR_SCL_GND 0x4c //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA219_ADDR_SCL_VS  0x4d //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA219_ADDR_SCL_SDA 0x4e //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA219_ADDR_SCL_SCL 0x4f //!< I2C address, A1 pin - SCL, A0 pin - SCL

/**
 * Bus voltage range
 */
typedef enum {
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum {
    INA219_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina219_gain_t;

/**
 * ADC resolution/averaging
 */
typedef enum {
    INA219_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA219_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA219_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA219_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA219_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA219_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA219_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA219_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA219_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA219_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA219_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina219_resolution_t;

/**
 * Operating mode
 */
typedef enum {
    INA219_MODE_POWER_DOWN = 0, //!< Power-done
    INA219_MODE_TRIG_SHUNT,     //!< Shunt voltage, triggered
    INA219_MODE_TRIG_BUS,       //!< Bus voltage, triggered
    INA219_MODE_TRIG_SHUNT_BUS, //!< Shunt and bus, triggered
    INA219_MODE_DISABLED,       //!< ADC off (disabled)
    INA219_MODE_CONT_SHUNT,     //!< Shunt voltage, continuous
    INA219_MODE_CONT_BUS,       //!< Bus voltage, continuous
    INA219_MODE_CONT_SHUNT_BUS  //!< Shunt and bus, continuous (default)
} ina219_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;

    uint16_t config;
    float i_lsb, p_lsb;
} ina219_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t ina219_init_desc(ina219_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_free_desc(ina219_t *dev);

/**
 * @brief Init device
 *
 * Read current device configuration into `dev->config`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_init(ina219_t *dev);

/**
 * @brief Reset device
 *
 * Same as power-on reset. Resets all registers to default values.
 * You still need to calibrate device to read current, otherwise
 * only shunt voltage readings will be valid.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_reset(ina219_t *dev);

/**
 * @brief Set device configuration
 *
 * @param dev Device descriptor
 * @param u_range Bus voltage range
 * @param gain Shunt voltage gain
 * @param u_res Bus voltage resolution and averaging
 * @param i_res Shunt voltage resolution and averaging
 * @param mode Device operational mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_configure(ina219_t *dev, ina219_bus_voltage_range_t u_range,
        ina219_gain_t gain, ina219_resolution_t u_res,
        ina219_resolution_t i_res, ina219_mode_t mode);

/**
 * @brief Get bus voltage range
 *
 * @param dev Device descriptor
 * @param[out] range Bus voltage range
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_range(ina219_t *dev, ina219_bus_voltage_range_t *range);

/**
 * @brief Get shunt voltage gain
 *
 * @param dev Device descriptor
 * @param[out] gain Shunt voltage gain
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_gain(ina219_t *dev, ina219_gain_t *gain);

/**
 * @brief Get bus voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Bus voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get shunt voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Shunt voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get operating mode
 *
 * @param dev Device descriptor
 * @param[out] mode Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_mode(ina219_t *dev, ina219_mode_t *mode);

/**
 * @brief Perform calibration
 *
 * Current readings will be valid only after calibration
 *
 * @param dev Device descriptor
 * @param r_shunt Shunt resistance, Ohm
 * @return `ESP_OK` on success
 */
esp_err_t ina219_calibrate(ina219_t *dev, float r_shunt);

/**
 * @brief Trigger single conversion
 *
 * Function will return an error if current operating
 * mode is not `INA219_MODE_TRIG_SHUNT`/`INA219_MODE_TRIG_BUS`/`INA219_MODE_TRIG_SHUNT_BUS`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_trigger(ina219_t *dev);

/**
 * @brief Read bus voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Bus voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read shunt voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Shunt voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read current
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] current Current, A
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_current(ina219_t *dev, float *current);

/**
 * @brief Read power
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] power Power, W
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_power(ina219_t *dev, float *power);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __INA219_H__ */
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/mqtt.h`

```text
#ifndef __MQTT_H__
#define __MQTT_H__

#include "mqtt_client.h"
#include "esp_wifi.h"

//static const char WIFI_SSID;
#define WIFI_SSID "IoT_bots"
//static const char WIFI_PASSWORD;
#define WIFI_PASSWORD "208208208"
static const char* MQTT_BROKER_URI;
static const char* MQTT_TAG;
static esp_mqtt_client_handle_t mqtt_client;;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_publish_task(void *arg);
void mqtt_app_start(void);
void mqtt_publish(void);
static float temp_value, battery_voltage, sys_current, sys_power;
void mqtt_update_temp (float temp);
void mqtt_update_battery_voltage (float voltage);
void mqtt_update_sys_current (float current);
void mqtt_update_sys_power (float power);
void mqtt_update_pwm_1 (int pwm);
void mqtt_update_pwm_2 (int pwm);
void mqtt_update_pwm_3 (int pwm);
void mqtt_update_pwm_4 (int pwm);
void sta_wifi_init(void);

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/espnow_config.h`

```text
#ifndef ESPNOW_CONFIG_H
#define ESPNOW_CONFIG_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)


typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;
/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

/* User defined fields of ESPNOW data struct. */
typedef struct {
    uint8_t type;                           // Broadcast or unicast ESPNOW data.
    uint8_t state;                          // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                       // Sequence number of ESPNOW data.
    uint16_t crc;                           // CRC16 value of ESPNOW data.
    uint32_t magic;                         // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[2];                     // Real payload of ESPNOW data.
    uint8_t mtr_a_pwm;
    uint8_t mtr_b_pwm;
    //float chip_temp;                        // ESP32-C3 chip temperature
    bool lights;                            // Lights ON/OFF
    uint8_t projection_x;
    uint8_t projection_y;
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                           //Send unicast ESPNOW data.
    bool broadcast;                         //Send broadcast ESPNOW data.
    uint8_t state;                          //Indicate that if has received broadcast ESPNOW data or not.
    uint8_t priority;                       // ESP-NOW device priority #
    uint32_t magic;                         //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                         //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                         //Delay between sending two ESPNOW data, unit: ms.
    int len;                                //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                        //Buffer pointing to ESPNOW data struct.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];     //MAC address of destination device. 
} espnow_send_param_t;

/*typedef struct {
    uint8_t     type;                       // Broadcast or unicast ESPNOW data.s
    uint16_t    seq_num;                     // Sequence number of ESPNOW data.
    uint16_t    crc;                         // CRC16 value of ESPNOW data.
    uint8_t     x_axis;
    uint8_t     y_axis;
    bool        nav_bttn;
    uint8_t     motor1_rpm_pcm;
    uint8_t     motor2_rpm_pcm;
    uint8_t     motor3_rpm_pcm;
    uint8_t     motor4_rpm_pcm;
} __attribute__((packed)) sensors_data_t;*/

// Struct holding sensors values
typedef struct {
    uint16_t    crc;                // CRC16 value of ESPNOW data
    int         x_axis;             // Joystick x-position
    int         y_axis;             // Joystick y-position
    bool        nav_bttn;           // Joystick push button
    uint8_t     motor1_rpm_pcm;     // PCMs for 4 motors
    uint8_t     motor2_rpm_pcm;
    uint8_t     motor3_rpm_pcm;
    uint8_t     motor4_rpm_pcm;
} __attribute__((packed)) sensors_data_t;

typedef struct {
    int len;                                // Length of ESPNOW data to be sent, unit: byte.
    uint8_t     *buffer;                      // Buffer; pointer to the data struct.
    uint8_t     dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} espnow_data_packet_t;

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/ina219.c`

```text
/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ina219.c
 *
 * ESP-IDF driver for INA219/INA220 Zerø-Drift, Bidirectional
 * Current/Power Monitor
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <math.h>
#include <esp_idf_lib_helpers.h>
#include "ina219.h"

#define I2C_FREQ_HZ 1000000 // Max 1 MHz for esp-idf, but supports up to 2.56 MHz

static const char *TAG = "ina219";

#define REG_CONFIG      0
#define REG_SHUNT_U     1
#define REG_BUS_U       2
#define REG_POWER       3
#define REG_CURRENT     4
#define REG_CALIBRATION 5

#define BIT_RST   15
#define BIT_BRNG  13
#define BIT_PG0   11
#define BIT_BADC0 7
#define BIT_SADC0 3
#define BIT_MODE  0

#define MASK_PG   (3 << BIT_PG0)
#define MASK_BADC (0xf << BIT_BADC0)
#define MASK_SADC (0xf << BIT_SADC0)
#define MASK_MODE (7 << BIT_MODE)
#define MASK_BRNG (1 << BIT_BRNG)

#define DEF_CONFIG 0x399f

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const float u_shunt_max[] = {
    [INA219_GAIN_1]     = 0.04,
    [INA219_GAIN_0_5]   = 0.08,
    [INA219_GAIN_0_25]  = 0.16,
    [INA219_GAIN_0_125] = 0.32,
};

static esp_err_t read_reg_16(ina219_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = (*val >> 8) | (*val << 8);

    return ESP_OK;
}

static esp_err_t write_reg_16(ina219_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = (val >> 8) | (val << 8);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_conf_bits(ina219_t *dev, uint16_t mask, uint8_t bit, uint16_t *res)
{
    uint16_t raw;
    CHECK(read_reg_16(dev, REG_CONFIG, &raw));

    *res = (raw & mask) >> bit;

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t ina219_init_desc(ina219_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr < INA219_ADDR_GND_GND || addr > INA219_ADDR_SCL_SCL)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t ina219_free_desc(ina219_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t ina219_init(ina219_t *dev)
{
    CHECK_ARG(dev);

    CHECK(read_reg_16(dev, REG_CONFIG, &dev->config));

    ESP_LOGD(TAG, "Initialize, config: 0x%04x", dev->config);

    return ESP_OK;
}

esp_err_t ina219_reset(ina219_t *dev)
{
    CHECK_ARG(dev);
    CHECK(write_reg_16(dev, REG_CONFIG, 1 << BIT_RST));

    dev->config = DEF_CONFIG;

    ESP_LOGD(TAG, "Device reset");

    return ESP_OK;
}

esp_err_t ina219_configure(ina219_t *dev, ina219_bus_voltage_range_t u_range,
        ina219_gain_t gain, ina219_resolution_t u_res,
        ina219_resolution_t i_res, ina219_mode_t mode)
{
    CHECK_ARG(dev);
    CHECK_ARG(u_range <= INA219_BUS_RANGE_32V);
    CHECK_ARG(gain <= INA219_GAIN_0_125);
    CHECK_ARG(u_res <= INA219_RES_12BIT_128S);
    CHECK_ARG(i_res <= INA219_RES_12BIT_128S);
    CHECK_ARG(mode <= INA219_MODE_CONT_SHUNT_BUS);

    dev->config = (u_range << BIT_BRNG) |
                  (gain << BIT_PG0) |
                  (u_res << BIT_BADC0) |
                  (i_res << BIT_SADC0) |
                  (mode << BIT_MODE);

    ESP_LOGD(TAG, "Config: 0x%04x", dev->config);

    return write_reg_16(dev, REG_CONFIG, dev->config);
}

esp_err_t ina219_get_bus_voltage_range(ina219_t *dev, ina219_bus_voltage_range_t *range)
{
    CHECK_ARG(dev && range);
    *range = 0;
    return read_conf_bits(dev, MASK_BRNG, BIT_BRNG, (uint16_t *)range);
}

esp_err_t ina219_get_gain(ina219_t *dev, ina219_gain_t *gain)
{
    CHECK_ARG(dev && gain);
    *gain = 0;
    return read_conf_bits(dev, MASK_PG, BIT_PG0, (uint16_t *)gain);
}

esp_err_t ina219_get_bus_voltage_resolution(ina219_t *dev, ina219_resolution_t *res)
{
    CHECK_ARG(dev && res);
    *res = 0;
    return read_conf_bits(dev, MASK_BADC, BIT_BADC0, (uint16_t *)res);
}

esp_err_t ina219_get_shunt_voltage_resolution(ina219_t *dev, ina219_resolution_t *res)
{
    CHECK_ARG(dev && res);
    *res = 0;
    return read_conf_bits(dev, MASK_SADC, BIT_SADC0, (uint16_t *)res);
}

esp_err_t ina219_get_mode(ina219_t *dev, ina219_mode_t *mode)
{
    CHECK_ARG(dev && mode);
    *mode = 0;
    return read_conf_bits(dev, MASK_MODE, BIT_MODE, (uint16_t *)mode);
}

esp_err_t ina219_calibrate(ina219_t *dev, float r_shunt)
{
    CHECK_ARG(dev);

    ina219_gain_t gain;
    CHECK(ina219_get_gain(dev, &gain));

    dev->i_lsb = (uint16_t)(u_shunt_max[gain] / r_shunt / 32767 * 100000000);
    dev->i_lsb /= 100000000;
    dev->i_lsb /= 0.0001;
    dev->i_lsb = ceil(dev->i_lsb);
    dev->i_lsb *= 0.0001;

    dev->p_lsb = dev->i_lsb * 20;

    uint16_t cal = (uint16_t)((0.04096) / (dev->i_lsb * r_shunt));

    ESP_LOGD(TAG, "Calibration: %.04f Ohm, 0x%04x", r_shunt, cal);

    return write_reg_16(dev, REG_CALIBRATION, cal);
}

esp_err_t ina219_trigger(ina219_t *dev)
{
    CHECK_ARG(dev);

    uint16_t mode = (dev->config & MASK_MODE) >> BIT_MODE;
    if (mode < INA219_MODE_TRIG_SHUNT || mode > INA219_MODE_TRIG_SHUNT_BUS)
    {
        ESP_LOGE(TAG, "Could not trigger conversion in this mode: %d", mode);
        return ESP_ERR_INVALID_STATE;
    }

    return write_reg_16(dev, REG_CONFIG, dev->config);
}

esp_err_t ina219_get_bus_voltage(ina219_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);

    uint16_t raw;
    CHECK(read_reg_16(dev, REG_BUS_U, &raw));

    *voltage = (raw >> 3) * 0.004;

    return ESP_OK;
}

esp_err_t ina219_get_shunt_voltage(ina219_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);

    int16_t raw;
    CHECK(read_reg_16(dev, REG_SHUNT_U, (uint16_t *)&raw));

    *voltage = raw / 100000.0;

    return ESP_OK;
}

esp_err_t ina219_get_current(ina219_t *dev, float *current)
{
    CHECK_ARG(dev && current);

    int16_t raw;
    CHECK(read_reg_16(dev, REG_CURRENT, (uint16_t *)&raw));

    *current = raw * dev->i_lsb;

    return ESP_OK;
}

esp_err_t ina219_get_power(ina219_t *dev, float *power)
{
    CHECK_ARG(dev && power);

    int16_t raw;
    CHECK(read_reg_16(dev, REG_POWER, (uint16_t *)&raw));

    *power = raw * dev->p_lsb;

    return ESP_OK;
}

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/controls.h`

```text
#ifndef CONTROLS_H
#define CONTROLS_H

/* 

To prevent assigning forbidden rpm values to the motor (i.e. to avoid short-citcuit) accidentially,
we define one struct tha tholds RPMs for four motors, as opposed to defining array of structs for single motor.

*/ 

/*

Struct that holds PCM for RPMs for each of 4 motors.
Positive PCM values for clock-wise rotation, and negative values for counter-vise rotation.

*/
struct motors_rpm {
    int motor1_rpm_pcm;
    int motor1_gpio;
    int motor2_rpm_pcm;
    int motor2_gpio;
    int motor3_rpm_pcm;
    int motor3_gpio;
    int motor4_rpm_pcm;
    int motor4_gpio;
};

//extern Motors *motors;

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/i2cdev.h`

```text
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file i2cdev.h
 * @defgroup i2cdev i2cdev
 * @{
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>

#ifdef __cplusplus
extern "C" {
#endif

#if HELPER_TARGET_IS_ESP8266

#define I2CDEV_MAX_STRETCH_TIME 0xffffffff

#else

#include <soc/i2c_reg.h>
#if defined(I2C_TIME_OUT_VALUE_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_VALUE_V
#elif defined(I2C_TIME_OUT_REG_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_REG_V
#else
#define I2CDEV_MAX_STRETCH_TIME 0x00ffffff
#endif

#endif /* HELPER_TARGET_IS_ESP8266 */

/**
 * I2C device descriptor
 */
typedef struct
{
    i2c_port_t port;         //!< I2C port number
    i2c_config_t cfg;        //!< I2C driver configuration
    uint8_t addr;            //!< Unshifted address
    SemaphoreHandle_t mutex; //!< Device mutex
    uint32_t timeout_ticks;  /*!< HW I2C bus timeout (stretch time), in ticks. 80MHz APB clock
                                  ticks for ESP-IDF, CPU ticks for ESP8266.
                                  When this value is 0, I2CDEV_MAX_STRETCH_TIME will be used */
} i2c_dev_t;

/**
 * I2C transaction type
 */
typedef enum {
    I2C_DEV_WRITE = 0, /**< Write operation */
    I2C_DEV_READ       /**< Read operation */
} i2c_dev_type_t;

/**
 * @brief Init library
 *
 * The function must be called before any other
 * functions of this library.
 *
 * @return ESP_OK on success
 */
esp_err_t i2cdev_init();

/**
 * @brief Finish work with library
 *
 * Uninstall i2c drivers.
 *
 * @return ESP_OK on success
 */
esp_err_t i2cdev_done();

/**
 * @brief Create mutex for device descriptor
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);

/**
 * @brief Delete mutex for device descriptor
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);

/**
 * @brief Take device mutex
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);

/**
 * @brief Give device mutex
 *
 * This function does nothing if option CONFIG_I2CDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);

/**
 * @brief Check the availability of the device
 *
 * Issue an operation of \p operation_type to the I2C device then stops.
 *
 * @param dev Device descriptor
 * @param operation_type Operation type
 * @return ESP_OK if device is available
 */
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type);

/**
 * @brief Read from slave device
 *
 * Issue a send operation of \p out_data register address, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_data Pointer to data to send if non-null
 * @param out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data,
        size_t out_size, void *in_data, size_t in_size);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_reg Pointer to register address to send if non-null
 * @param out_reg_size Size of register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg,
        size_t out_reg_size, const void *out_data, size_t out_size);

/**
 * @brief Read from register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_read().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg,
        void *in_data, size_t in_size);

/**
 * @brief Write to register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_write().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg,
        const void *out_data, size_t out_size);

#define CONFIG_I2CDEV_TIMEOUT (1000)

#define I2C_DEV_TAKE_MUTEX(dev) do { \
        esp_err_t __ = i2c_dev_take_mutex(dev); \
        if (__ != ESP_OK) return __;\
    } while (0)

#define I2C_DEV_GIVE_MUTEX(dev) do { \
        esp_err_t __ = i2c_dev_give_mutex(dev); \
        if (__ != ESP_OK) return __;\
    } while (0)

#define I2C_DEV_CHECK(dev, X) do { \
        esp_err_t ___ = X; \
        if (___ != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(dev); \
            return ___; \
        } \
    } while (0)

#define I2C_DEV_CHECK_LOGE(dev, X, msg, ...) do { \
        esp_err_t ___ = X; \
        if (___ != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return ___; \
        } \
    } while (0)

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __I2CDEV_H__ */
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/ultrasonic.h`

```text
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "driver/gpio.h"
#include "esp_err.h"

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000
#define ROUNDTRIP_M 5800.0f
#define ROUNDTRIP_CM 58

typedef struct {
    gpio_num_t trigger_gpio;  // GPIO for the trigger pin
    gpio_num_t echo_gpio;     // GPIO for the echo pin
} ultrasonic_sensor_t;

esp_err_t ultrasonic_init (const ultrasonic_sensor_t *sensor);
esp_err_t ultrasonic_measure_raw (const ultrasonic_sensor_t *sensor,uint32_t max_time_us, uint32_t *time_us);

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/motor_controls.h`

```text
#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLS_H

// Interpolate value (x) based on raw reading, min/max limits.
/*

    Joystick scale:     4096    2048        0
    PWM scale:          8191    4096        0

    PWM Output:         +8191       0   -8191
*/
static int interpolate_raw_val (int raw) {
    int x;

    x = raw*2;

    return x;
}
// Function that converts raw value from joystick scale (0 to 4096) to PCM scale (-8192 to 8192).
static int rescale_raw_val (int raw) {

    int s;
    s = 4*raw - 8190;
    //s = -(raw*raw) + 4.122*raw - 8190;
    //s = (8190/2048^1)*(raw - 2048)^1;   // Linear rescaling
    //s = (8190/1870^3)*(raw - 1870)^3;   // Cubic rescaling
    return s;
}

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/mqtt.c`

```text
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"

#include "mqtt.h"

//static const char* MQTT_BROKER_URI = "mqtt://10.100.50.16:1883";//74.14.210.168";//"mqtt://mqtt.techquadbit.net";
static const char* MQTT_BROKER_URI = "mqtt://74.14.210.168";//"mqtt://mqtt.techquadbit.net";

static const char* MQTT_TAG = "MQTT_Robot";
static esp_mqtt_client_handle_t mqtt_client = NULL;

static float temp_value = 0.0f;
static float battery_voltage = 0.0f;
static float sys_current = 0.0f;
static float sys_power = 0.0f;
static int pwm_1 = 0, pwm_2 = 0, pwm_3 = 0, pwm_4 = 0;

static void mqtt_publish_task(void *arg) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)arg;
    
    while (1) {
        //float tsens_value = 0.0f;
        //temperature_sensor_get_celsius(temp_sensor, &tsens_value);
        ESP_LOGW("ESP32-C3", "Temperature value %.02f ℃", temp_value);
        char temp_str[6], battery_voltage_str[8], sys_current_str[8], sys_power_str[8];
        char pwm_1_str[5], pwm_2_str[5], pwm_3_str[5], pwm_4_str[5];
        snprintf(temp_str, sizeof(temp_str), "%.02f", temp_value);
        snprintf(battery_voltage_str, sizeof(battery_voltage_str), "%.02f", battery_voltage);
        snprintf(sys_current_str, sizeof(sys_current_str), "%.02f", sys_current);
        snprintf(sys_power_str, sizeof(sys_power_str), "%.02f", sys_power);
        snprintf(pwm_1_str, sizeof(pwm_1_str), "%d", pwm_1);
        snprintf(pwm_2_str, sizeof(pwm_2_str), "%d", pwm_2);
        snprintf(pwm_3_str, sizeof(pwm_3_str), "%d", pwm_3);
        snprintf(pwm_4_str, sizeof(pwm_4_str), "%d", pwm_4);

        // Publish a message every 5 seconds
        esp_mqtt_client_publish(mqtt_client, "/bitrider/temp", temp_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/battery_voltage", battery_voltage_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/sys_current", sys_current_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/sys_power", sys_power_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-1", pwm_1_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-2", pwm_2_str, 0, 1, 0);
        /*esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-3", pwm_3_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/bitrider/pwm-4", pwm_4_str, 0, 1, 0);*/
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(MQTT_TAG, "Called task to publish topic /bitrider/temp");
    }
}

void mqtt_update_temp (float temp) { temp_value = temp; }
void mqtt_update_battery_voltage (float voltage) { battery_voltage = voltage; }
void mqtt_update_sys_current (float current) { sys_current = current; }
void mqtt_update_sys_power (float power) { sys_power = power; }
void mqtt_update_pwm_1 (int pwm) { pwm_1 = pwm; }
void mqtt_update_pwm_2 (int pwm) { pwm_2 = pwm; }
void mqtt_update_pwm_3 (int pwm) { pwm_3 = pwm; }
void mqtt_update_pwm_4 (int pwm) { pwm_4 = pwm; }

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_publish(client, "/esp/test", "Hello from ESP32-C3!", 0, 1, 0);
            mqtt_client = client;
            xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 8192, NULL, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

void mqtt_publish() {
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(NULL);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_mqtt_client_start(client);
    
    // Publish a message
    esp_mqtt_client_publish(client, "/esp/test", "Hello from Alex!", 0, 1, 0);
    
    // Stop the client
    esp_mqtt_client_stop(client);
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/*void sta_wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}*/

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/config.h`

```text
#ifndef CONFIG_H
#define CONFIG_H

// MOTORS PWM CONFIG
#define MTR_FREQUENCY               (7000)                  // 1000
#define MTR_MODE                    LEDC_LOW_SPEED_MODE
#define MTR_DUTY_RES                LEDC_TIMER_13_BIT       // 13-bit resolution supports maximum duty value 8192 (8)
// LEFT SIDE MOTORS, FORWARD
#define MTR_FRONT_LEFT_IO           (6)
#define MTR_FRONT_LEFT_TMR          LEDC_TIMER_0
#define MTR_FRONT_LEFT              LEDC_CHANNEL_1
#define MTR_FRONT_LEFT_DUTY         (3361)
// RIGHT SIDE MOTORS, FORWARD
#define MTR_FRONT_RIGHT_IO          (5)
#define MTR_FRONT_RIGHT_TMR         LEDC_TIMER_1
#define MTR_FRONT_RIGHT             LEDC_CHANNEL_0
#define MTR_FRONT_RIGHT_DUTY        (3361)
// LEFT SIDE MOTORS, REVERSE
#define MTR_FRONT_LEFT_REV_IO       (4)
#define MTR_FRONT_LEFT_REV_TMR      LEDC_TIMER_2
#define MTR_FRONT_LEFT_REV          LEDC_CHANNEL_2
#define MTR_FRONT_LEFT_REV_DUTY     (3361)
// RIGHT SIDE MOTORS, REVERSE
#define MTR_FRONT_RIGHT_REV_IO      (7)
#define MTR_FRONT_RIGHT_REV_TMR     LEDC_TIMER_3
#define MTR_FRONT_RIGHT_REV         LEDC_CHANNEL_3
#define MTR_FRONT_RIGHT_REV_DUTY    (3361)


//#define LEDC_DUTY                   (3361) //7820) // 8068, 7944, 7820, 7696, 7572, *7680*, 7424, 7168, 6144, 512, 768
//#define LEDC_FREQUENCY              (2500) //8192) //4000) // For LED the freuqncy of 500Hz seems to be sufficient. // Frequency in Hertz. For DC motor, set frequency at 5 kHz; try 1kHz @ 14 bits resolution


#endif

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/rc.h`

```text
#ifndef RC_H
#define RC_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "controls.h"
#include "config.h"

#define ADC_CHNL            ADC_CHANNEL_1
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC1_CHAN0          ADC1_CHANNEL_0
#define ADC1_CHAN1          ADC1_CHANNEL_1

//static const char *TAG = "ESP IDF Robot"
struct motors_rpm m;
static int adc_raw[2][10];
static int voltage[2][10];
static int s = 0, sample = 5, x = 0, y = 0, x_sum = 0, y_sum = 0;

bool do_calibration1_chan0, do_calibration1_chan1;

adc_cali_handle_t adc1_cali_chan0_handle, adc1_cali_chan1_handle;
adc_oneshot_unit_handle_t adc1_handle;
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
static int interpolate_raw_val (int raw);
static int rescale_raw_val (int raw);

static esp_err_t rc_adc_init (void) {
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK( adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,//ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//    
    adc1_cali_chan0_handle = NULL;
    adc1_cali_chan1_handle = NULL;
    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);
    do_calibration1_chan1 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN1, ADC_ATTEN, &adc1_cali_chan1_handle);

    return ESP_OK;
}

static int check_motor_pcm(int x) {
    int lim = 7440;
    if (x > lim)
        return 8190;//lim;
    else if (x < -lim)
        return -8190;//-lim;
    else
        return x;
}

// Update PWM based on received values
// IMPORTANT: x and y values correspod to the PWM!
static void update_pwm (int rc_x, int rc_y) {
    
    x = check_motor_pcm(rescale_raw_val(rc_x));
    y = check_motor_pcm(rescale_raw_val(rc_y));
    //ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);

    /*if (s < sample) {
        x_sum += check_motor_pcm(rescale_raw_val(x));
        y_sum += check_motor_pcm(rescale_raw_val(y));
        s ++;
    }
    else if (s == sample) {
        //x = check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        //y = check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        x = x_sum / sample;
        y = y_sum / sample;
        s++;*/

    /*
    
    (+, +0)     (+, +)      (+0, +)
    (-, +)      (0, 0)      (+, -)
    (-, -0)     (-, -)      (-0, -)
    
    if (1024 < x < 2048 && 1024 < y < 2048) {}
    */

    // ADDED ON AUG 6, 2025: to be tested!
    // CONTINOUS UPDATE
    int x_val = x;
    int y_val = y;
    int x_centered = x_val - 2048;
    int y_centered = x_val - 2048;
    // Map joystick to motor direction from y-axis
    int motor_a_dir = y_centered >= 0 ? 1 : -1;
    int motor_b_dir = y_centered >= 0 ? 1 : -1;
    int motor_a_speed = abs(y_centered) * 8192 / 2048;
    int motor_b_speed = abs(y_centered) * 8192 / 2048;
    // Add turning effect from x-axis
    motor_a_speed -= x_centered * 8192 / 2048;
    motor_b_speed += x_centered * 8192 / 2048;
    // Clamp speeds
    if (motor_a_speed < 0) motor_a_speed = 0;
    if (motor_b_speed < 0) motor_b_speed = 0;
    if (motor_a_speed > 8192) motor_a_speed = 8192;
    if (motor_b_speed > 8192) motor_b_speed = 8192;
    //set_motor_direction();
    //set_motor_speed();
    // Pass PWM values to the proper DC motors depending on the joystick y-axis position
    // Forward
    /*if (y_val > y_centered) {
        m.motor1_rpm_pcm = motor_a_speed;
        m.motor2_rpm_pcm = motor_b_speed;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    // Reverse
    if (y_val < y_centered) {
        m.motor1_rpm_pcm = motor_a_speed;
        m.motor2_rpm_pcm = motor_b_speed;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }*/


    /*
    // Turn Left
    if (x == 8190 && y == -8190) {
        m.motor1_rpm_pcm = 6172;
        m.motor2_rpm_pcm = 8190;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if (x == 8190 && y == 8190) {
        m.motor1_rpm_pcm = 8190;
        m.motor2_rpm_pcm = 6172;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if (x == -8190 && y == -8190) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 6172;
        m.motor4_rpm_pcm = 8190;
    }
    else if (x == -8190 && y == 8190) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 8190;
        m.motor4_rpm_pcm = 6172;
    }*/
    // FORWARD AND REVERSE
    //if ((x > 1500) && (y > 700 && y < 850)) {
    
    else if ((x > 1500) && (y > -2500 && y < 2500)) {
        //ESP_LOGW("ESP-NOW", "FORWARD");
        // Both sides rotate in forward direction.
        m.motor1_rpm_pcm = x;   // left, forward
        m.motor2_rpm_pcm = x;   // right, forward
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    //else if ((x < 0) && (y > 700 && y < 850)) {
    else if ((x < 0) && (y > -2500 && y < 2500)) {
        //ESP_LOGW("ESP-NOW", "REVERSE");
        // Both sides rotate in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -x;
        m.motor4_rpm_pcm = -x;
    }
    // ROTATE CLOCKWISE AND COUNTER CLOCKWISE
    else if ((x > -2500 && x < 2500) && (y < 0)) {
        //ESP_LOGW("ESP-NOW", "LEFT");
        // Left side rotates in forward direction, right side rotates in reverse direction.
        m.motor1_rpm_pcm = 0;//-y;
        m.motor2_rpm_pcm = -y;//0;
        m.motor3_rpm_pcm = 0;//-y;
        m.motor4_rpm_pcm = -y;//0;
    }
    else if ((x > -2500 && x < 2500) && (y > 900)) {
        //ESP_LOGW("ESP-NOW", "RIGHT");
        // Right side rotates in forward direction, left side rotates in reverse direction.
        m.motor1_rpm_pcm = y;//0;
        m.motor2_rpm_pcm = 0;//y; 
        m.motor3_rpm_pcm = y;//0;
        m.motor4_rpm_pcm = 0;//y; 
    }
    else {
        //ESP_LOGW("ESP-NOW", "STAND STILL");
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

}

static void update_motors_pwm (int pwm_motor_1, int pwm_motor_2) {

    /* UPDATED MOTOR LOGIC */
    if (pwm_motor_1 >= 0 && pwm_motor_2 >= 0) {
        m.motor1_rpm_pcm = pwm_motor_1;
        m.motor2_rpm_pcm = pwm_motor_2;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    /*if (pwm_motor_1 > 0 && pwm_motor_2 < 0) {
        m.motor1_rpm_pcm = pwm_motor_1;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -pwm_motor_2;
        m.motor4_rpm_pcm = 0;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 > 0) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = -pwm_motor_1;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = pwm_motor_2;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 < 0) {
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -pwm_motor_1;
        m.motor4_rpm_pcm = -pwm_motor_2;
    }*/

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    ESP_LOGW("MTR LGC", "M1: %d, M2: %d, M3: %d, M4: %d",
        m.motor1_rpm_pcm,
        m.motor2_rpm_pcm,
        m.motor3_rpm_pcm,
        m.motor4_rpm_pcm);
}

/*static void rc_get_raw_data() {

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][1]));
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0, adc_raw[0][0]);
    ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1, adc_raw[0][1]);
    ESP_LOGI("Joystick L/R", "Position: %d (%d)", rescale_raw_val(adc_raw[0][0]),  check_motor_pcm(rescale_raw_val(adc_raw[0][0])));
    ESP_LOGI("Joystick F", "Position: %d (%d)", rescale_raw_val(adc_raw[0][1]), check_motor_pcm(rescale_raw_val(adc_raw[0][1])));
    ESP_LOGW("Joystick", " sample %d, (x,y): (%d, %d)", sample, x, y);

    if (s < sample) {
        x_sum += check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        y_sum += check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        s ++;
    }
    else if (s == sample) {
        //x = check_motor_pcm(rescale_raw_val(adc_raw[0][0]));
        //y = check_motor_pcm(rescale_raw_val(adc_raw[0][1]));
        x = x_sum / sample;
        y = y_sum / sample;
    //x = buf.x_axis;
    //y = buf.y_axis;
    

    if ((x > 0 && x < 500) && (y > 500)) {
        ESP_LOGW("RC", "FORWARD");
        // Both sides rotate in forward direction.
        m.motor1_rpm_pcm = y;   // left, forward
        m.motor2_rpm_pcm = y;   // right, forward
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    else if ((x > 0 && x < 500) && (y < -200)) {
        ESP_LOGW("RC", "REVERSE");
        // Both sides rotate in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -y;
        m.motor4_rpm_pcm = -y;
    }
    else if ((y < 0 && y > -200) && (x < -1000)) {
        ESP_LOGW("RC", "LEFT");
        // Left side rotates in forward direction, right side rotates in reverse direction.
        m.motor1_rpm_pcm = -x;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = -x;
        m.motor4_rpm_pcm = 0;
    }
    else if ((y < 0 && y > -200) && (x > 1000)) {
        ESP_LOGW("RC", "RIGHT");
        // Right side rotates in forward direction, left side rotates in reverse direction.
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = x; 
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = x; 
    }
    else {
        ESP_LOGW("RC", "STAND STILL");
        m.motor1_rpm_pcm = 0;
        m.motor2_rpm_pcm = 0;
        m.motor3_rpm_pcm = 0;
        m.motor4_rpm_pcm = 0;
    }
    s++;
    }
    else {
        x_sum = 0;
        y_sum = 0;
        s = 0;
    }

    ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
    ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);
    ESP_LOGI("PWM", "Motor 3 PWM: %d", m.motor3_rpm_pcm);
    ESP_LOGI("PWM", "Motor 4 PWM: %d", m.motor4_rpm_pcm);

    if (do_calibration1_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
    }
    if (do_calibration1_chan1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        ESP_LOGI("ESP IDF Robot", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN1, voltage[0][1]);
    }
}*/

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ESP IDF Robot", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ESP IDF Robot", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    #endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("ESP IDF Robot", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("ESP IDF Robot", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("ESP IDF Robot", "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI("ESP IDF Robot", "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI("ESP IDF Robot", "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    #endif
}

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/esp_idf_lib_helpers.h`

```text
/*
 * Copyright (c) 2019 Tomoyuki Sakurai <y@trombik.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#if !defined(__ESP_IDF_LIB_HELPERS__H__)
#define __ESP_IDF_LIB_HELPERS__H__

/* XXX this header file does not need to include freertos/FreeRTOS.h.
 * but without it, ESP8266 RTOS SDK does not include `sdkconfig.h` in correct
 * order. as this header depends on sdkconfig.h, sdkconfig.h must be included
 * first. however, the SDK includes this header first, then includes
 * `sdkconfig.h` when freertos/FreeRTOS.h is not explicitly included. an
 * evidence can be found in `build/${COMPONENT}/${COMPONENT}.d` in a failed
 * build.
 */
#include <freertos/FreeRTOS.h>
#include <esp_idf_version.h>

#if !defined(ESP_IDF_VERSION) || !defined(ESP_IDF_VERSION_VAL)
#error Unknown ESP-IDF/ESP8266 RTOS SDK version
#endif

/* Minimal supported version for ESP32, ESP32S2 */
#define HELPER_ESP32_MIN_VER    ESP_IDF_VERSION_VAL(3, 3, 5)
/* Minimal supported version for ESP8266 */
#define HELPER_ESP8266_MIN_VER  ESP_IDF_VERSION_VAL(3, 3, 0)

/* HELPER_TARGET_IS_ESP32
 * 1 when the target is esp32
 */
#if defined(CONFIG_IDF_TARGET_ESP32) \
        || defined(CONFIG_IDF_TARGET_ESP32S2) \
        || defined(CONFIG_IDF_TARGET_ESP32S3) \
        || defined(CONFIG_IDF_TARGET_ESP32C2) \
        || defined(CONFIG_IDF_TARGET_ESP32C3) \
        || defined(CONFIG_IDF_TARGET_ESP32C5) \
        || defined(CONFIG_IDF_TARGET_ESP32C6) \
        || defined(CONFIG_IDF_TARGET_ESP32P4) \
        || defined(CONFIG_IDF_TARGET_ESP32C61) \
        || defined(CONFIG_IDF_TARGET_ESP32H2)
#define HELPER_TARGET_IS_ESP32     (1)

/* HELPER_TARGET_IS_ESP8266
 * 1 when the target is esp8266
 */
#elif defined(CONFIG_IDF_TARGET_ESP8266)
#define HELPER_TARGET_IS_ESP8266   (1)
#else
#error BUG: cannot determine the target
#endif

#if HELPER_TARGET_IS_ESP32 && ESP_IDF_VERSION < HELPER_ESP32_MIN_VER
#error Unsupported ESP-IDF version. Please update!
#endif

#if HELPER_TARGET_IS_ESP8266 && ESP_IDF_VERSION < HELPER_ESP8266_MIN_VER
#error Unsupported ESP8266 RTOS SDK version. Please update!
#endif

/* HELPER_SPI_HOST_DEFAULT
 *
 * The default SPI_HOST for spi_host_device_t
 */
#if CONFIG_IDF_TARGET_ESP32
#define HELPER_SPI_HOST_DEFAULT HSPI_HOST
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define HELPER_SPI_HOST_DEFAULT SPI2_HOST
#elif CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define HELPER_SPI_HOST_DEFAULT SPI1_HOST
#elif CONFIG_IDF_TARGET_ESP32H2
#define HELPER_SPI_HOST_DEFAULT SPI1_HOST
#elif CONFIG_IDF_TARGET_ESP32P4
#define HELPER_SPI_HOST_DEFAULT SPI1_HOST
#endif

/* show the actual values for debugging */
#if DEBUG
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32C3))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32H2))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32S2))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP8266))
#pragma message(VAR_NAME_VALUE(ESP_IDF_VERSION_MAJOR))
#endif

#endif
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/ultrasonic.c`

```text
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_timer.h>

#include "ultrasonic.h"

esp_err_t ultrasonic_init (const ultrasonic_sensor_t *sensor)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure the trigger GPIO
    gpio_config_t trigger_config = {
        .pin_bit_mask = (1ULL << sensor->trigger_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&trigger_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure the echo GPIO
    gpio_config_t echo_config = {
        .pin_bit_mask = (1ULL << sensor->echo_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&echo_config);
    
    return ret;
}

esp_err_t ultrasonic_measure_raw (const ultrasonic_sensor_t *sensor, uint32_t max_time_us, uint32_t *time_us)
{
    if (sensor == NULL || time_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send a 10us pulse to the trigger pin
    gpio_set_level(sensor->trigger_gpio, 1);
    vTaskDelay (10 / portTICK_PERIOD_MS); 
    gpio_set_level(sensor->trigger_gpio, 0);

    // Wait for the echo pin to go high
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_gpio) == 0) {
        if (esp_timer_get_time() - start_time > max_time_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    // Measure the duration of the high signal on the echo pin
    start_time = esp_timer_get_time();
    while (gpio_get_level(sensor->echo_gpio) == 1) {
        if (esp_timer_get_time() - start_time > max_time_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    *time_us = esp_timer_get_time() - start_time;

    return ESP_OK;
}
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/i2cdev.c`

```text
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file i2cdev.c
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "i2cdev.h"

static const char *TAG = "i2cdev";

typedef struct {
    SemaphoreHandle_t lock;
    i2c_config_t config;
    bool installed;
} i2c_port_state_t;

static i2c_port_state_t states[I2C_NUM_MAX];

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_TAKE(port)
#else
#define SEMAPHORE_TAKE(port) do { \
        if (!xSemaphoreTake(states[port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT))) \
        { \
            ESP_LOGE(TAG, "Could not take port mutex %d", port); \
            return ESP_ERR_TIMEOUT; \
        } \
        } while (0)
#endif

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_GIVE(port)
#else
#define SEMAPHORE_GIVE(port) do { \
        if (!xSemaphoreGive(states[port].lock)) \
        { \
            ESP_LOGE(TAG, "Could not give port mutex %d", port); \
            return ESP_FAIL; \
        } \
        } while (0)
#endif

esp_err_t i2cdev_init()
{
    memset(states, 0, sizeof(states));

#if !CONFIG_I2CDEV_NOLOCK
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        states[i].lock = xSemaphoreCreateMutex();
        if (!states[i].lock)
        {
            ESP_LOGE(TAG, "Could not create port mutex %d", i);
            return ESP_FAIL;
        }
    }
#endif

    return ESP_OK;
}

esp_err_t i2cdev_done()
{
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!states[i].lock) continue;

        if (states[i].installed)
        {
            SEMAPHORE_TAKE(i);
            i2c_driver_delete(i);
            states[i].installed = false;
            SEMAPHORE_GIVE(i);
        }
#if !CONFIG_I2CDEV_NOLOCK
        vSemaphoreDelete(states[i].lock);
#endif
        states[i].lock = NULL;
    }
    return ESP_OK;
}

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] creating mutex", dev->addr, dev->port);

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not create device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] deleting mutex", dev->addr, dev->port);

    vSemaphoreDelete(dev->mutex);
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] taking mutex", dev->addr, dev->port);

    if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take device mutex", dev->addr, dev->port);
        return ESP_ERR_TIMEOUT;
    }
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] giving mutex", dev->addr, dev->port);

    if (!xSemaphoreGive(dev->mutex))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not give device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

inline static bool cfg_equal(const i2c_config_t *a, const i2c_config_t *b)
{
    return a->scl_io_num == b->scl_io_num
        && a->sda_io_num == b->sda_io_num
#if HELPER_TARGET_IS_ESP32
        && a->master.clk_speed == b->master.clk_speed
#elif HELPER_TARGET_IS_ESP8266
        && ((a->clk_stretch_tick && a->clk_stretch_tick == b->clk_stretch_tick) 
            || (!a->clk_stretch_tick && b->clk_stretch_tick == I2CDEV_MAX_STRETCH_TIME)
        ) // see line 232
#endif
        && a->scl_pullup_en == b->scl_pullup_en
        && a->sda_pullup_en == b->sda_pullup_en;
}

static esp_err_t i2c_setup_port(const i2c_dev_t *dev)
{
    if (dev->port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;

    esp_err_t res;
    if (!cfg_equal(&dev->cfg, &states[dev->port].config) || !states[dev->port].installed)
    {
        ESP_LOGD(TAG, "Reconfiguring I2C driver on port %d", dev->port);
        i2c_config_t temp;
        memcpy(&temp, &dev->cfg, sizeof(i2c_config_t));
        temp.mode = I2C_MODE_MASTER;

        // Driver reinstallation
        if (states[dev->port].installed)
        {
            i2c_driver_delete(dev->port);
            states[dev->port].installed = false;
        }
#if HELPER_TARGET_IS_ESP32
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
        // See https://github.com/espressif/esp-idf/issues/10163
        if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
            return res;
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
#else
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
        if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
            return res;
#endif
#endif
#if HELPER_TARGET_IS_ESP8266
        // Clock Stretch time, depending on CPU frequency
        temp.clk_stretch_tick = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
        if ((res = i2c_driver_install(dev->port, temp.mode)) != ESP_OK)
            return res;
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
#endif
        states[dev->port].installed = true;

        memcpy(&states[dev->port].config, &temp, sizeof(i2c_config_t));
        ESP_LOGD(TAG, "I2C driver successfully reconfigured on port %d", dev->port);
    }
#if HELPER_TARGET_IS_ESP32
    int t;
    if ((res = i2c_get_timeout(dev->port, &t)) != ESP_OK)
        return res;
    // Timeout cannot be 0
    uint32_t ticks = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
    if ((ticks != t) && (res = i2c_set_timeout(dev->port, ticks)) != ESP_OK)
        return res;
    ESP_LOGD(TAG, "Timeout: ticks = %" PRIu32 " (%" PRIu32 " usec) on port %d", dev->timeout_ticks, dev->timeout_ticks / 80, dev->port);
#endif

    return ESP_OK;
}

esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1 | (operation_type == I2C_DEV_READ ? 1 : 0), true);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));

        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);

    return res;
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (out_data && out_size)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, dev->addr << 1, true);
            i2c_master_write(cmd, (void *)out_data, out_size, true);
        }
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
        i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));

        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true);
        if (out_reg && out_reg_size)
            i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
        i2c_master_write(cmd, (void *)out_data, out_size, true);
        i2c_master_stop(cmd);
        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
    return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
    return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}
```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/main/blink_example_main.c`

```text
/* Robot Controls
    Generate PWM signals to control motors.

    By:         Alexander Bobkov
    Date:       Dec 21, 2024

    Updated:    Jan 10, 2025
                Jun 26, 2025
                Jul 26, 2025 (ESP-IDF + MQTT + WiFi)
                Aug 6 , 2025 Continous interpolation of joystick x- and y- values
                Jan 3,  2026 Revised motor control logic

    built-in LED GPIO:          10
    build-in push button GPIO:  3

    ESP-PDF: v5.4.1

    SPECS

    Voltage, DevBoard:  5V
    Voltage, Robot:     7.4V (2S LiPo battery)
    Current, standby:   300mA
    Current, motors:    850mA
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/temperature_sensor.h"
//#include "driver/mcpwm.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
/* ADC */
#include "rc.h"
/* Motors Controls */
#include "motor_controls.h"
/* System-wide controls */
#include "controls.h"
#include "esp_adc/adc_continuous.h"

/* ESP-NOW */
#include <string.h>
#include <assert.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "mqtt.h"
#include "esp_system.h"
#include "espnow_config.h"

#include "ultrasonic.h"
#include "ina219.h"

#include "config.h"

static const char *TAG = "ESP IDF Robot";

#define I2C_PORT 0
#define I2C_ADDR 0x40
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 3
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 2
#define CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM 100


/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

// Retrieve values from configuration menu
#define BLINK_GPIO                  CONFIG_BLINK_GPIO       // 10 GPIO of on-board LED
#define PUSH_BTN_GPIO               CONFIG_BUTTON_GPIO      // 8, not 3 GPIO of on-board push-button
#define MTR_FL_GPIO                 0 //CONFIG_MOTOR_FRONT_LEFT_GPIO
// ADC
#define ADC_ATTEN                   ADC_ATTEN_DB_11
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_UNIT                    ADC_UNIT_1
#define ADC_CONV_MODE               ADC_CONV_BOTH_UNIT// ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2    // ESP32C3
#define READ_LEN                    1024//256
//#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
//#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button
#define _ADC_UNIT_STR(unit)         #unit
#define ADC_UNIT_STR(unit)          _ADC_UNIT_STR(unit)
uint32_t x_avg = 0, y_avg = 0;
static TaskHandle_t led_task_handle;
static TaskHandle_t s_task_handle;
static TaskHandle_t m_task_handle;  // Task for controlling motors PWMs
static adc_channel_t channel[2] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
static sensors_data_t buf;

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_INPUT_PIN_SEL ((1ULL<<PUSH_BTN_GPIO) | (1ULL<<NAV_BTN))
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<BLINK_GPIO))

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

static temperature_sensor_handle_t temp_sensor;
static temperature_sensor_config_t temp_sensor_config;
float tsens_value;
static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t s_led_state = 1;


/*  ============================
            ESP NOW
    ============================

    ESP32-C3 Blue board MAC:        54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:         34:b7:da:f9:33:8d
    ESP32-C3 Breadboard #1 MAC:     e4:b0:63:17:9e:45
    ESP32-C3 Breadboard #2 MAC:     9c:9e:6e:14:b5:54 (rapid blink)
    ESP32-C3 zenBoard MAC:          dc:da:0c:8e:b3:70
*/
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0x54, 0x32, 0x04, 0x46, 0x71, 0x80};     // MAC address of troubleshooting Dev board
//static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};
static uint8_t robot_mac[ESP_NOW_ETH_ALEN]      = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
static uint8_t rc_mac[ESP_NOW_ETH_ALEN]         = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control
static uint8_t espnow_seq[ESPNOW_DATA_MAX]      = {0, 0};

static int rc_x = 0, rc_y = 0;
static int pwm_motor_1 = 0;
static int pwm_motor_2 = 0;
static int pwm_motor_3 = 0;
static int pwm_motor_4 = 0;

//uint8_t broadcastAddress[] = {};
//struct_message controlData;
esp_now_peer_info_t peerInfo;
static void espnow_deinit(espnow_send_param_t *send_param);

static void blink_led(void) {
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void IRAM_ATTR gpio_isr_handler (void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
// Push button interrupt task
static void gpio_task (void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}
static void nav_key_task (void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void configure_button (void) {
    ESP_LOGI(TAG, "Configured on-board push button");
    //gpio_reset_pin(PUSH_BTN_GPIO);
    //gpio_set_direction(PUSH_BTN_GPIO, GPIO_MODE_INPUT);
}

// Struct for storing DC Motors PWM values
static void motors_init (void) {
    m.motor1_rpm_pcm = 0;
    m.motor2_rpm_pcm = 0;
    m.motor3_rpm_pcm = 0;
    m.motor4_rpm_pcm = 0;
}

// Initialize DC Motors PWM timers and channels
static void ledc_init (void) {

    // MOTOR FRONT RIGHT, FORWARD
    ledc_timer_config_t ledc_timer_1 = {
        .speed_mode =       MTR_MODE,// LEDC_MODE,
        .duty_resolution =  MTR_DUTY_RES,// LEDC_DUTY_RES,
        .timer_num =        MTR_FRONT_RIGHT_TMR,// LEDC_TIMER,
        .freq_hz =          MTR_FREQUENCY,// LEDC_FREQUENCY,
        .clk_cfg =          LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_1));
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode =       MTR_MODE,
        .channel =          MTR_FRONT_RIGHT,// LEDC_CHANNEL_0,// MTR_FRONT_RIGHT,
        .timer_sel =        MTR_FRONT_RIGHT_TMR,// LEDC_TIMER,
        .intr_type =        LEDC_INTR_DISABLE,
        .gpio_num =         MTR_FRONT_RIGHT_IO,
        .duty =             MTR_FRONT_RIGHT_DUTY,
        .hpoint =           0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
    // MOTOR FRONT LEFT, FORWARD
    ledc_timer_config_t ledc_timer_2 = {
        .speed_mode =       MTR_MODE,
        .duty_resolution =  MTR_DUTY_RES,
        .timer_num =        MTR_FRONT_LEFT_TMR,
        .freq_hz =          MTR_FREQUENCY,
        .clk_cfg =          LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_2));
    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode =       MTR_MODE,
        .channel =          MTR_FRONT_LEFT,
        .timer_sel =        MTR_FRONT_LEFT_TMR,
        .intr_type =        LEDC_INTR_DISABLE,
        .gpio_num =         MTR_FRONT_LEFT_IO,
        .duty =             MTR_FRONT_LEFT_DUTY,
        .hpoint =           0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_2));

    // MOTOR FRONT RIGHT, REVERSE
    ledc_timer_config_t ledc_timer_3 = {
        .speed_mode =       MTR_MODE,// LEDC_MODE,
        .duty_resolution =  MTR_DUTY_RES,// LEDC_DUTY_RES,
        .timer_num =        MTR_FRONT_RIGHT_REV_TMR,// LEDC_TIMER,
        .freq_hz =          MTR_FREQUENCY,// LEDC_FREQUENCY,
        .clk_cfg =          LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_3));
    ledc_channel_config_t ledc_channel_3 = {
        .speed_mode =       MTR_MODE,
        .channel =          MTR_FRONT_RIGHT_REV,// LEDC_CHANNEL_0,// MTR_FRONT_RIGHT,
        .timer_sel =        MTR_FRONT_RIGHT_REV_TMR,// LEDC_TIMER,
        .intr_type =        LEDC_INTR_DISABLE,
        .gpio_num =         MTR_FRONT_RIGHT_REV_IO,
        .duty =             MTR_FRONT_RIGHT_REV_DUTY,
        .hpoint =           0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_3));
    // MOTOR FRONT LEFT, REVERSE
    ledc_timer_config_t ledc_timer_4 = {
        .speed_mode =       MTR_MODE,
        .duty_resolution =  MTR_DUTY_RES,
        .timer_num =        MTR_FRONT_LEFT_REV_TMR,
        .freq_hz =          MTR_FREQUENCY,
        .clk_cfg =          LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_4));
    ledc_channel_config_t ledc_channel_4 = {
        .speed_mode =       MTR_MODE,
        .channel =          MTR_FRONT_LEFT_REV,
        .timer_sel =        MTR_FRONT_LEFT_REV_TMR,
        .intr_type =        LEDC_INTR_DISABLE,
        .gpio_num =         MTR_FRONT_LEFT_REV_IO,
        .duty =             MTR_FRONT_LEFT_REV_DUTY,
        .hpoint =           0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_4));
}

/* ESP-NOW */
// Wi-Fi should start before using ESP-NOW
static void wifi_init()
{
    /*
    * STAND-ALONE
    */
    /*ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    #endif*/

    /*
    * WI-FI
    */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));//ESPNOW_WIFI_MODE));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,//"IoT_bots2",
            .password = WIFI_PASSWORD,// "208208208",
        },
    };
    ESP_ERROR_CHECK (esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void led_task (void *arg) {
    while(1) {
        //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
	    gpio_set_level(BLINK_GPIO, s_led_state);
	    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        s_led_state = !s_led_state;
	}
}
static void temp_sensor_task (void *arg) {
    while (true) {
        ESP_LOGI("ESP32-C3", "Reading sensor temperature");
        float tsens_value;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
        ESP_LOGW("ESP32-C3", "Temperature value %.02f ℃", tsens_value);
        mqtt_update_temp (tsens_value); 
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
/* UPDATED MOTOR LOGIC */
static float clampf (float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}
void joystick_mix (int X_raw, int Y_raw, int *pwm_a, int *pwm_b) {
    // 1. Normalize joystick to [-1 .. +1]
    float x = (float)(X_raw - 1020) / 1020.0f; float y = (float)(Y_raw - 1020) / 1020.0f;
    
    // 2. Steering gain for smooth arcs
    const float k = 0.4f;
    
    // 3. Raw differential mix
    float L0 = y + k * x;
    float R0 = y - k * x;
    
    // 4. Normalize pair so neither exceeds magnitude 1
    float m = fmaxf(1.0f, fmaxf(fabsf(L0), fabsf(R0)));
    float L = L0 / m;
    float R = R0 / m;
    
    // 5. Scale to signed PWM range [-8191 .. +8190]
    float L_scaled = L * 8190.0f;
    float R_scaled = R * 8190.0f;
    
    // 6. Clamp and output as integers
    *pwm_a = (int)clampf(L_scaled, -8191.0f, 8190.0f);
    *pwm_b = (int)clampf(R_scaled, -8191.0f, 8190.0f);
}
// Task function to read joystick values and update motors rotation speeds.
static void rc_task (void *arg) {
    while (true) {
        // update_pwm (rc_x, rc_y);     // Orginal motor update logic
        joystick_mix (rc_y, rc_x, &pwm_motor_1, &pwm_motor_2);
        update_motors_pwm (pwm_motor_1, pwm_motor_2);   // Revised motor update logic
        //ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);
        vTaskDelay (100 / portTICK_PERIOD_MS);  // Determines responsiveness
        //vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}
static void display_xy() {
    while (true) {
        ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);
        ESP_LOGI("PWM", "M1: %d, M2: %d, M3: %d, M4: %d", 
            m.motor1_rpm_pcm, m.motor2_rpm_pcm, 
            m.motor3_rpm_pcm, m.motor4_rpm_pcm);
        joystick_mix (rc_y, rc_x, &pwm_motor_1, &pwm_motor_2);
        ESP_LOGI("Converted PWM", "M1+M2: %d, M3+M4: %d", pwm_motor_1, pwm_motor_2);
        uint8_t channel;
        wifi_band_t band;
        esp_wifi_get_channel(&channel, NULL);
        esp_wifi_get_band(&band);
        ESP_LOGI(TAG, "Wi-Fi Channel: %d, Band: %d", channel, band);

        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}

/*
    EXP32-C3 Chip built-in temprature sensor
    Read & display the temperature value
*/
static void chip_sensor_init () {
    temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}
static void display_chip_temperature () {
    ESP_LOGI("ESP32-C3", "Reading sensor temperature");
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
    ESP_LOGW("ESP32-C3", "Temperature value %.02f ℃", tsens_value);
}

// ESP-NOW callback on data received
void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len) {

    memcpy(&buf, data, sizeof(buf));    // Write buffer into the struct
    rc_x = buf.x_axis;                  // Save joystic x-axis value
    rc_y = buf.y_axis;                  // Save joystic y-axis value
    // Update motors PWM values from received data
    m.motor1_rpm_pcm = buf.motor1_rpm_pcm;
    m.motor2_rpm_pcm = buf.motor2_rpm_pcm;
    m.motor3_rpm_pcm = buf.motor3_rpm_pcm;
    m.motor4_rpm_pcm = buf.motor4_rpm_pcm;
    // Update motors PWM values using joystick x- and y-axis values
    update_pwm(rc_x, rc_y);
    mqtt_update_pwm_1(rc_x);            // Publish PWM-1 on MQTT Broker
    mqtt_update_pwm_2(rc_y);            // Publish PWM-2 on MQTT Broker
}

void ultrasonic_task (void *arg) {
    ultrasonic_sensor_t sensor = {
        .trigger_gpio = GPIO_NUM_4,  // Example GPIO for trigger
        .echo_gpio = GPIO_NUM_5       // Example GPIO for echo
    };
    ESP_ERROR_CHECK(ultrasonic_init(&sensor));

    uint32_t time_us;
    while (true) {
        ESP_ERROR_CHECK(ultrasonic_measure_raw(&sensor, PING_TIMEOUT, &time_us));
        float distance_cm = (float)time_us / ROUNDTRIP_CM;
        ESP_LOGI(TAG, "Distance: %.2f cm", distance_cm);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task(void *pvParameters)
{
    ina219_t dev;
    memset(&dev, 0, sizeof(ina219_t));

    assert(CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM > 0);
    ESP_ERROR_CHECK(ina219_init_desc(&dev, I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&dev));

    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&dev, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
            INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));

    ESP_LOGI(TAG, "Calibrating INA219");

    ESP_ERROR_CHECK(ina219_calibrate(&dev, (float)CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    float bus_voltage, shunt_voltage, current, power;

    ESP_LOGI(TAG, "Starting the loop");
    while (1)
    {
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&dev, &bus_voltage));
        ESP_ERROR_CHECK(ina219_get_shunt_voltage(&dev, &shunt_voltage));
        ESP_ERROR_CHECK(ina219_get_current(&dev, &current));
        ESP_ERROR_CHECK(ina219_get_power(&dev, &power));
        /* Using float in printf() requires non-default configuration in
         * sdkconfig. See sdkconfig.defaults.esp32 and
         * sdkconfig.defaults.esp8266  */
        printf("VBUS: %.04f V, VSHUNT: %.04f mV, IBUS: %.04f mA, PBUS: %.04f mW\n",
                bus_voltage, shunt_voltage * 1000, current * 1000, power * 1000);

        uint8_t channel;
        wifi_band_t band;
        esp_wifi_get_channel(&channel, NULL);
        esp_wifi_get_band(&band);
        printf("Wi-Fi Channel: %d, Band:%d\n", channel, band);
        //ESP_LOGE(TAG, "Wi-Fi Channel: %d, Band: %d", channel, band);
        mqtt_update_battery_voltage(bus_voltage);
        mqtt_update_sys_current(1000.00*current);
        mqtt_update_sys_power(power);

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

void app_main(void)
{
    // Initialize NVS before Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // MOTORS
    motors_init();

    // Use wifi_init() for ESP-NOW and Wi-Fi setup
    wifi_init();

    // Initialize internal temperature sensor
    chip_sensor_init();
    xTaskCreate(temp_sensor_task, "ESP32C3 Sensor", 2048, NULL, 15, NULL);

    // Initialize LED
    ledc_init();
    int var = 8191;
    gpio_config_t io_conf = {};

    // Configure on-board LED
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    xTaskCreate(led_task, "LED", 2048, NULL, 15, NULL);

    // Configure on-board push button
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(PUSH_BTN_GPIO, GPIO_INTR_NEGEDGE);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "GPIO task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(PUSH_BTN_GPIO, gpio_isr_handler, (void*) PUSH_BTN_GPIO);

    // Configure navigation button
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(NAV_BTN, GPIO_INTR_NEGEDGE);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(nav_key_task, "NAV Keys task", 2048, NULL, 10, NULL);
    gpio_isr_handler_add(NAV_BTN, gpio_isr_handler, (void*) NAV_BTN);

    configure_button();
    printf("Added button interrupt");

    mqtt_app_start();
    // Initialize buffer with 0s
    buf.x_axis = 0;
    buf.y_axis = 0;
    buf.motor1_rpm_pcm = 0;
    esp_now_init();
    esp_now_register_recv_cb((void*)onDataReceived);

    // ADC
    rc_adc_init();
    xTaskCreate(rc_task, "RC", 2048, NULL, 5, NULL);

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(display_xy, "coordinates", configMINIMAL_STACK_SIZE * 8, NULL, 4, NULL);
}

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/CMakeLists.txt`

```text
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP-IDF_Robot)

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/scan_code.py`

```text
import os

# Configure this to your project root
PROJECT_ROOT = "/home/alex/github/ESP-Nodes/ESP-IDF_Robot"
OUTPUT_MD = "project_sources.md"

# Folders to scan (relative to project root)
TARGET_FOLDERS = [
    "main",
]

# File extensions to include
SOURCE_EXTENSIONS = {
    ".c", ".h", ".cpp", ".hpp",
    ".py", ".txt", ".cmake",
}

# Special filenames to include even without extension
SPECIAL_FILES = {
    "CMakeLists.txt",
}

def is_source_file(filename):
    # Special cases
    if filename in SPECIAL_FILES:
        return True

    # Extension-based detection
    _, ext = os.path.splitext(filename)
    return ext in SOURCE_EXTENSIONS

def collect_sources(root):
    sources = []

    # Scan only selected folders
    for folder in TARGET_FOLDERS:
        full_path = os.path.join(root, folder)
        if not os.path.exists(full_path):
            continue

        for dirpath, _, filenames in os.walk(full_path):
            for f in filenames:
                if is_source_file(f):
                    sources.append(os.path.join(dirpath, f))

    # Also include top-level files
    for f in os.listdir(root):
        if is_source_file(f):
            sources.append(os.path.join(root, f))

    return sources

def write_markdown(sources, output_file):
    with open(output_file, "w", encoding="utf-8") as md:
        md.write("# Project Source Archive\n\n")
        md.write("Generated automatically for analysis.\n\n")

        for path in sources:
            md.write(f"## File: `{path}`\n\n")
            md.write("```text\n")

            try:
                with open(path, "r", encoding="utf-8", errors="replace") as src:
                    md.write(src.read())
            except Exception as e:
                md.write(f"[Error reading file: {e}]")

            md.write("\n```\n\n")

if __name__ == "__main__":
    sources = collect_sources(PROJECT_ROOT)
    write_markdown(sources, OUTPUT_MD)
    print(f"Markdown document generated: {OUTPUT_MD}")

```

## File: `/home/alex/github/ESP-Nodes/ESP-IDF_Robot/pytest_blink.py`

```text
# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import logging
import os

import pytest
from pytest_embedded_idf.dut import IdfDut


@pytest.mark.supported_targets
@pytest.mark.generic
def test_blink(dut: IdfDut) -> None:
    # check and log bin size
    binary_file = os.path.join(dut.app.binary_path, 'blink.bin')
    bin_size = os.path.getsize(binary_file)
    logging.info('blink_bin_size : {}KB'.format(bin_size // 1024))

```

