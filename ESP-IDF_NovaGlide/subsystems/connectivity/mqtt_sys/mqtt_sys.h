#ifndef MQTT_SYS_H
#define MQTT_SYS_H

#include "freertos/FreeRTOS.h"
#include "mqtt_client.h"

#define MQTT_BROKER_URI "mqtt://74.14.210.168"
#define WIFI_SSID "IoT_bots"
#define WIFI_PASSWORD "208208208"

// Forward declaration
typedef struct mqtt_system_t mqtt_system_t;

// Struct definition
struct mqtt_system_t {
    float temp_value;
    float battery_voltage;
    float sys_current;
    float sys_power;
    int pwm_left;
    int pwm_right;

    esp_mqtt_client_handle_t client;

    void (*update)(mqtt_system_t *self, TickType_t now);
};

void mqtt_system_init(mqtt_system_t *sys);
void mqtt_update_temp(mqtt_system_t *sys, float temp);
void mqtt_update_battery(mqtt_system_t *sys, float voltage);
void mqtt_update_current(mqtt_system_t *sys, float current);
void mqtt_update_power(mqtt_system_t *sys, float power);
void mqtt_update_pwm(mqtt_system_t *sys, int left, int right);

#endif  // <-- This was missing!
