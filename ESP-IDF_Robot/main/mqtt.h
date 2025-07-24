#ifndef __MQTT_H__
#define __MQTT_H__

#include "mqtt_client.h"
#include "esp_wifi.h"

static const char* WIFI_SSID;
static const char* WIFI_PASSWORD;
static const char* MQTT_BROKER_URI;
static const char* SERVICE_TAG;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

#endif