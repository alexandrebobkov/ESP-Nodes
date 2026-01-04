#include "mqtt_sys.h"
#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT_SYS";
static esp_mqtt_client_handle_t client = NULL;

static void mqtt_publish_temp_impl(mqtt_system_t *self, float t)
{
    self->temp = t;
    if (client) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.2f", t);
        esp_mqtt_client_publish(client, "/novaglides/temp", buf, 0, 1, 0);
    }
}

static void mqtt_publish_pwm_impl(mqtt_system_t *self, int p1, int p2)
{
    self->pwm_1 = p1;
    self->pwm_2 = p2;
}

static void mqtt_update_impl(mqtt_system_t *self, TickType_t now)
{
    (void)self;
    (void)now;
}

void mqtt_system_init(mqtt_system_t *sys)
{
    sys->temp = 0;
    sys->pwm_1 = 0;
    sys->pwm_2 = 0;

    sys->publish_temp = mqtt_publish_temp_impl;
    sys->publish_pwm = mqtt_publish_pwm_impl;
    sys->update = mqtt_update_impl;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtt://192.168.1.10",
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_start(client);
}
