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
float temp_value;
void mqtt_publish_temp (float temp);
void sta_wifi_init(void);

#endif