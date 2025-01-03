/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"

#include "rc.h"
#include "motor_controls.h"
#include "controls.h"

#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button

/*  
    ============================
            ESP NOW
    ============================

    ESP32-C3 Luatos ESP32C3 board MAC:      54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:                 34:b7:da:f9:33:8d
    ESP32-C3 Breadboard MAC:                e4:b0:63:17:9e:45
*/

static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};
//static uint8_t receiver_mac[ESP_NOW_ETH_ALEN]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;
static uint8_t flagToSend = 0;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t robot_mac[ESP_NOW_ETH_ALEN]      = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
static uint8_t rc_mac[ESP_NOW_ETH_ALEN]         = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control
static sensors_data_t *buf;
static sensors_data_t *buffer;

static void rc_send_data_task2 (void *pvParameter);

#define ESPNOW_MAXDELAY 512

static const char *TAG = "Remote Controller";

static QueueHandle_t s_example_espnow_queue;

// Broadcast address
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45 };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = 1; // UNICAST IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    //esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    //memcpy(buf->payload, (uint8_t)16, send_param->len - sizeof(example_espnow_data_t));
    //memcpy(buf->payload[0], (uint8_t)12, send_param->len - sizeof(example_espnow_data_t));
    //memcpy(buf->payload[0], 12, send_param->len - sizeof(example_espnow_data_t));
    buf->payload[0] = (uint8_t)12;
    buf->payload[1] = (uint8_t)10;
    ESP_LOGW(TAG, "Payload: %x", (uint8_t)buf->payload);
    ESP_LOGW(TAG, "payload[0]: %x", (uint8_t)buf->payload[0]);
    ESP_LOGW(TAG, "payload[1]: %x", (uint8_t)buf->payload[1]);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true; //false;
    send_param->broadcast = false; //true;
    send_param->state = 0;
    //esp_random();
    send_param->magic = 50;//esp_random(); //(uint32_t)50;//esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

static void rc_task (void *arg) {
    while (true) {
        rc_get_raw_data();

        ESP_LOGI("PWM", "Motor 1 PWM: %d", m.motor1_rpm_pcm);
        ESP_LOGI("PWM", "Motor 2 PWM: %d", m.motor2_rpm_pcm);
        ESP_LOGI("PWM", "Motor 3 PWM: %d", m.motor3_rpm_pcm);
        ESP_LOGI("PWM", "Motor 4 PWM: %d", m.motor4_rpm_pcm);

        //vTaskDelay (10 / portTICK_PERIOD_MS);  // Determines responsiveness  
        vTaskDelay (1000 / portTICK_PERIOD_MS); 
    }
}
void deletePeer (void) {
    uint8_t delStatus = esp_now_del_peer(receiver_mac);
    if (delStatus != 0) {
        ESP_LOGE("ESP-NOW", "Could not delete peer");
    }
}

/*
    ESP-NOW
*/
/* Prepare ESPNOW data to be sent. */
void sensors_data_prepare(espnow_data_packet_t *send_packet)
{
    //sensors_data_t *buffer;
    //malloc(sizeof(sensors_data_t));
    //send_packet->buffer = &buffer;
    //sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    sensors_data_t *buffer = (sensors_data_t *)send_packet->buffer;
    assert(send_packet->len >= sizeof(sensors_data_t));

    buffer->type = 1;
    buffer->crc = 0;
    buffer->x_axis = 0;
    buffer->y_axis = 0;
    buffer->nav_bttn = 0;
    buffer->motor1_rpm_pcm = 0;
    buffer->motor2_rpm_pcm = 0;
    buffer->motor3_rpm_pcm = 0;
    buffer->motor4_rpm_pcm = 0;
    ESP_LOGW(TAG, "x-axis: %x", (uint8_t)buffer->x_axis);
    buffer->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buffer, send_packet->len);
}

void sendData (void) {
    // Send data, specify receiver MAC address, pointer to the data being sent, and length of data being sent.
    //sensors_data_t *buffer;
    buffer = malloc(sizeof(sensors_data_t));
    buffer->type = 1;
    buffer->crc = 0;
    buffer->x_axis = (uint8_t)240;
    buffer->y_axis = (uint8_t)2040;
    buffer->nav_bttn = 0;
    buffer->motor1_rpm_pcm = (uint8_t)10;
    buffer->motor2_rpm_pcm = 0;
    buffer->motor3_rpm_pcm = 0;
    buffer->motor4_rpm_pcm = 0;
    ESP_LOGI(TAG, "x-axis: 0x%04X", (uint8_t)buffer->x_axis);
    ESP_LOGI(TAG, "y-axis: 0x%04X", (uint8_t)buffer->y_axis);
    ESP_LOGI(TAG, "pcm 1: 0x%04X", (uint8_t)buffer->motor1_rpm_pcm);
    ESP_LOGI(TAG, "pcm 2: 0x%04X", (uint8_t)buffer->motor2_rpm_pcm);
    ESP_LOGI(TAG, "pcm 3: 0x%04X", (uint8_t)buffer->motor3_rpm_pcm);
    ESP_LOGI(TAG, "pcm 4: 0x%04X", (uint8_t)buffer->motor4_rpm_pcm);

    //uint8_t result = esp_now_send(receiver_mac, &flagToSend, sizeof(flagToSend));
    uint8_t result = esp_now_send(receiver_mac, &buffer, sizeof(buffer));
    if (result != 0) {
        ESP_LOGE("ESP-NOW", "Error sending data!");
        deletePeer();
    }
    else
        ESP_LOGW("ESP-NOW", "Data was sent.");
}
static void rc_send_data_task (void *arg) {

    while (true) {
        flagToSend = !flagToSend;
        if (esp_now_is_peer_exist(receiver_mac)) {
            sendData();
        }
        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}

static void rc_send_data_task2 (void *pvParameter) {

    espnow_data_packet_t *send_packet = (espnow_data_packet_t *)pvParameter;

    while (true) {
        //memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
        esp_err_t r = esp_now_send(receiver_mac, send_packet->buffer, sizeof(sensors_data_t));//send_packet->len);
        //esp_now_send(send_packet->dest_mac, send_packet->buffer, send_packet->len);

        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Send error.");
            vTaskDelete(NULL);
            break;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
static esp_err_t rc_espnow_init (void) {

    espnow_data_packet_t *send_packet;

    send_packet = malloc(sizeof(espnow_data_packet_t));
    if (send_packet == NULL) {
        ESP_LOGE(TAG, "malloc fail.");
        return ESP_FAIL;
    }

    memset(send_packet, 0, sizeof(espnow_data_packet_t));
    memcpy(send_packet->dest_mac, receiver_mac, ESP_NOW_ETH_ALEN);
    send_packet->len = CONFIG_ESPNOW_SEND_LEN; // 128
    send_packet->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    sensors_data_prepare(send_packet);
    xTaskCreate(rc_send_data_task2, "controller data packets task", 2048, send_packet, 8, NULL);

    return ESP_OK;
}

void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

    //memcpy(buf, data, data_len);
    buf = (sensors_data_t*)data;
    ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04x", buf->motor1_rpm_pcm);
}


void app_main(void)
{
    /*
        ADC
    */
   //rc_adc_init();
   //xTaskCreate(rc_task, "RC", 2048, NULL, 5, NULL);

   // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    esp_now_init();
    esp_now_register_recv_cb(onDataReceived);

    memcpy (peerInfo.peer_addr, receiver_mac, 6);
    esp_now_add_peer(&peerInfo);
    if (esp_now_is_peer_exist(receiver_mac)) {
        ESP_LOGI("ESP-NOW", "Receiver exists.");
        sendData();
    }
    else
        ESP_LOGE("ESP-NOW", "Receiver does not exists.");
    xTaskCreate (rc_send_data_task, "RC", 2048, NULL, 15, NULL);

    //rc_espnow_init();

    /*
    example_wifi_init();
    example_espnow_init();
    */
}
