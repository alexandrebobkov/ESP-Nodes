#ifndef __MQTTRONIX_H__
#define __MQTTRONIX_H__

static const char* MQTT_BROKER_URI = "mqtt://mqtt.techquadbit.net";
static const char* MQTT_TAG = "MQTTronix";
//static const char WIFI_SSID;
#define WIFI_SSID "IoT_bots"
//static const char WIFI_PASSWORD;
#define WIFI_PASSWORD "208208208"
static const char* MQTT_BROKER_URI;
//static const char* MQTT_TAG;
//static esp_mqtt_client_handle_t mqtt_client = NULL;

void mqttronix_update_temp (float temp);

/*static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
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
void sta_wifi_init(void);*/

#endif