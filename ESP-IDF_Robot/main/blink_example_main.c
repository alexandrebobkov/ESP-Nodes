/* Robot Controls
    Generate PWM signals to control motors.

    By: Alexander Bobkov
    Date: Dec 21, 2024

    built-in LED GPIO: 10
    build-in push button GPIO: 3
*/
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
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
//#include "driver/adc.h"
//#include "esp_adc_cal.h"
//#include "esp_adc/adc_oneshot.h"
//#include "esp_adc/adc_cali.h"
//#include "esp_adc/adc_cali_scheme.h"
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
//#include "espnow_utils.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "espnow_config.h"

#include "config.h"

static const char *TAG = "ESP IDF Robot";


/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

// Retrieve values from configuration menu
#define BLINK_GPIO                  CONFIG_BLINK_GPIO       // 10 GPIO of on-board LED
#define PUSH_BTN_GPIO               CONFIG_BUTTON_GPIO      // 3 GPIO of on-board push-button
#define MTR_FL_GPIO                 0 //CONFIG_MOTOR_FRONT_LEFT_GPIO
// ADC
#define ADC_ATTEN                   ADC_ATTEN_DB_11
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_UNIT                    ADC_UNIT_1
#define ADC_CONV_MODE               ADC_CONV_BOTH_UNIT// ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2    // ESP32C3
#define READ_LEN                    1024//256
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button
#define _ADC_UNIT_STR(unit)         #unit
#define ADC_UNIT_STR(unit)          _ADC_UNIT_STR(unit)
uint32_t x_avg = 0, y_avg = 0;
static TaskHandle_t led_task_handle;
static TaskHandle_t s_task_handle;
static TaskHandle_t m_task_handle;  // Task for controlling motors PWMs
//static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
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

// Declare struct to hold motors RPMs
//Motors *m;

static temperature_sensor_handle_t temp_sensor;
static temperature_sensor_config_t temp_sensor_config;
static float tsens_value;

static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t s_led_state = 1;
//static bool s_led_state = true;

/*  ============================
            ESP NOW
    ============================

    ESP32-C3 Blue board MAC:    54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:     34:b7:da:f9:33:8d
    ESP32-C3 Breadboard MAC:    e4:b0:63:17:9e:45
*/
#define ESPNOW_MAXDELAY 512
static QueueHandle_t espnow_queue;
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0x54, 0x32, 0x04, 0x46, 0x71, 0x80};     // MAC address of troubleshooting Dev board
//static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN]  = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};
static uint8_t robot_mac[ESP_NOW_ETH_ALEN]      = {0xE4, 0xB0, 0x63, 0x17, 0x9E, 0x45};     // MAC address of Robot
static uint8_t rc_mac[ESP_NOW_ETH_ALEN]         = {0x34, 0xB7, 0xDA, 0xF9, 0x33, 0x8D};     // MAC address of Remote Control
static uint8_t espnow_seq[ESPNOW_DATA_MAX]      = {0, 0};
//static uint8_t espnow_broadcast_mac[ESP_NOW_ETH_ALEN] = {};
typedef struct struct_message {
    char node[32];
    uint8_t motor_a_pwm;
} struct_message;

static int rc_x = 0, rc_y = 0;

uint8_t broadcastAddress[] = {};
struct_message controlData;
esp_now_peer_info_t peerInfo;
static void espnow_deinit(espnow_send_param_t *send_param);

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

#else
#error "unsupported LED type"
#endif

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
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void configure_button (void) {
    ESP_LOGI(TAG, "Configured on-board push button");
    //gpio_reset_pin(PUSH_BTN_GPIO);
    //gpio_set_direction(PUSH_BTN_GPIO, GPIO_MODE_INPUT);
}

static void motors_init (void) {

    m.motor1_rpm_pcm = 0;
    m.motor2_rpm_pcm = 0;
    m.motor3_rpm_pcm = 0;
    m.motor4_rpm_pcm = 0;

}

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
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    //ESP_ERROR_CHECK( esp_wifi_set_mode(CONFIG_ESPNOW_WIFI_MODE_STATION_SOFTAP) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}
static void espnow_send_cb (const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB; //EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}
static void espnow_recv_cb (const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
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
    }
    else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }
    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    // Display payload received.
    ESP_LOGW(TAG, "Received payload: %x", (unsigned int)recv_cb->data);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}
/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);
    // Display received data.
    ESP_LOGW(TAG, "Received data size: %d", data_len);
    ESP_LOGW(TAG, "Payload: %x", (uint8_t)buf->payload);
    ESP_LOGW(TAG, "payload[0] = %x", (uint8_t)buf->payload[0]);
    ESP_LOGW(TAG, "payload[1] = %x", (uint8_t)buf->payload[1]);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

void espnow_data_prepare(espnow_send_param_t *send_param) {
    // Data struct
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    buf->payload[0] = (uint8_t)3;
    buf->payload[1] = (uint8_t)3;
    //esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    ESP_LOGW(TAG, "payload[0]: %x", buf->payload[0]);
    ESP_LOGW(TAG, "payload[1]: %x", buf->payload[1]);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_task (void *pvParameter) {
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;
    esp_err_t task_status;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    // Retrieve send parameters passed as reference.
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    // Send data to the destination MAC address.
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            // Send Callback
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));
                // Copy destination MAC address to the parameters struct.
                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                // Append data struct to the parameters struct.
                espnow_data_prepare(send_param);
                
                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                /*
                task_status = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
                if (task_status != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }*/
                break;
            }
            // Receive callback.
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                /*      
                ========    BROADCAST   ========
                */
                // If data was sent to all devices (broadcast)
                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            espnow_deinit(send_param);
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
                            espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                /*      
                ========    UNICAST   ========
                */
                // If data was sent to the specific device (unicast)
                else if (ret == ESPNOW_DATA_UNICAST) {
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

static esp_err_t espnow_init(void) {
    espnow_send_param_t *send_param;

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    // Confirm that queue exists, and continue if so.
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create ESP-NOW mutex failed.");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key in menuconfig. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    // The higher the magic number is, the lower the priority of the device is
    // higher number -> receiver
    send_param->magic = 55;//esp_random();   // Arbitrary number that determines which device is sender/receiver.
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    // Maximum data length is ESP_NOW_MAX_DATA_LEN = 250
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN); // malloc(sizeof(message)); // malloc(CONFIG_ESPNOW_SEND_LEN);    // Data to be sent?
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(espnow_task, "robot_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void motors_task (void *pvParameter) {}
static void led_task (void *arg) {
    while(1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
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
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
// Task function to read joystick values and update motors rotation speeds.
static void rc_task (void *arg) {
    while (true) {
        //rc_get_raw_data();    // Uncomment to read on-board joystick
        update_pwm (rc_x, rc_y);

        /*ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
        ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

        ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
        ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);*/

        ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);
        /*ESP_LOGW(TAG, "Motor 1 PWM: %d", m.motor1_rpm_pcm);
        ESP_LOGW(TAG, "Motor 2 PWM: %d", m.motor2_rpm_pcm);
        ESP_LOGW(TAG, "Motor 3 PWM: %d", m.motor3_rpm_pcm);
        ESP_LOGW(TAG, "Motor 4 PWM: %d", m.motor4_rpm_pcm);*/

        //vTaskDelay (10 / portTICK_PERIOD_MS);  // Determines responsiveness  
        vTaskDelay (1000 / portTICK_PERIOD_MS); 
    }
}

/*
    EXP32-C3 Chip built-in temprature sensor
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

void onDataReceived (const uint8_t *mac_addr, const uint8_t *data, uint8_t data_len) {

    memcpy(&buf, data, sizeof(buf));
    //buf = (sensors_data_t*)data;
    /*ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04X", buf->x_axis);
    ESP_LOGI(TAG, "y-axis: 0x%04X", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04X", buf->motor1_rpm_pcm);*/

    /*
    ESP_LOGW(TAG, "Data was received (%i bytes)", data_len);
    ESP_LOGI("Received (x,y)", "( %d, %d )", buf.x_axis, buf.y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04X", buf.motor1_rpm_pcm);
    ESP_LOGI(TAG, "PCM 2: 0x%04X", buf.motor2_rpm_pcm);
    ESP_LOGI(TAG, "PCM 3: 0x%04X", buf.motor3_rpm_pcm);
    ESP_LOGI(TAG, "PCM 4: 0x%04X", buf.motor4_rpm_pcm);
    */
    rc_x = buf.x_axis;
    rc_y = buf.y_axis;

    //update_pwm(buf.x_axis, buf.y_axis);
    update_pwm(rc_x, rc_y);
    /*ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);*/
    
}

void app_main(void)
{
    // Initialize internal temperature sensor
    chip_sensor_init();
    xTaskCreate(temp_sensor_task, "ESP32C3 Sensor", 2048, NULL, 15, NULL);

    // Initialize LED
    // Used to control the DC motor
    ledc_init();
    int var = 8191;
    // Initialize the config structure.
    gpio_config_t io_conf = {};

    /* Configure the peripheral according to the LED type */
    //configure_led();

    /* 
        Configure on-board LED
    */
    /*io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);*/
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    //xTaskCreate(led_task, "LED task", 1024, NULL, 5, NULL);
    xTaskCreate(led_task, "LED", 2048, NULL, 15, NULL);

    // Configure on-board push button
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // Set push button interrupt
    gpio_set_intr_type(PUSH_BTN_GPIO, GPIO_INTR_NEGEDGE);//ANYEDGE);
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
    // Set navigation button interrupt
    gpio_set_intr_type(NAV_BTN, GPIO_INTR_NEGEDGE);//ANYEDGE);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(nav_key_task, "NAV Keys task", 2048, NULL, 10, NULL);
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(NAV_BTN, gpio_isr_handler, (void*) NAV_BTN);

    configure_button();
    //configure_dc_mc();
    printf("Added button interrupt");

    // ESP-NOW
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    //wifi_init();        // is it required when using ESP-NOW?
    //espnow_init();
    //esp_now_add_peer(&peerInfo);

    buf.x_axis = 0;
    buf.y_axis = 0;
    buf.motor1_rpm_pcm = 0;
    wifi_init();
    esp_now_init();
    esp_now_register_recv_cb(onDataReceived);

    /*
        ADC
    */
    rc_adc_init();
    xTaskCreate(rc_task, "RC", 2048, NULL, 5, NULL);

    /*
        MOTORS
    */
    motors_init();
    //xTaskCreate(motors_task, "PWM task", 2048, NULL, 10, NULL);

    //while (1) {
        //display_chip_temperature();

        //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

        // ADC
        // Display GPIOs used        
        //ESP_LOGI(TAG, "ADC1_CH0: %d", ADC1_CHANNEL_0);
        //ESP_LOGI(TAG, "ADC1_CH1: %d", ADC1_CHANNEL_1);

        //rc_get_raw_data();

    //    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

        // Gradually reduce DC motor rotation speed.
        /*if (var >= 2048)
            var -= 124;
        else
            var = 8091;
        //update_pwm(var);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, var);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);*/

        /*
        ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, m.motor1_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);
        ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, m.motor2_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

        ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, m.motor3_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);
        ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, m.motor4_rpm_pcm);
        ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

        ESP_LOGW(TAG, "Motor 1 PWM: %d", m.motor1_rpm_pcm);
        ESP_LOGW(TAG, "Motor 2 PWM: %d", m.motor2_rpm_pcm);
        ESP_LOGW(TAG, "Motor 3 PWM: %d", m.motor3_rpm_pcm);
        ESP_LOGW(TAG, "Motor 4 PWM: %d", m.motor4_rpm_pcm);
        */
    //}
}
