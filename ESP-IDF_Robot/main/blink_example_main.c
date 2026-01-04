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
        update_pwm (rc_x, rc_y);     // Orginal motor update logic
        //joystick_mix (rc_y, rc_x, &pwm_motor_1, &pwm_motor_2);
        update_motors_pwm (pwm_motor_1, pwm_motor_2);   // Revised motor update logic
        ESP_LOGI("x,y", "( %d, %d ) [ %d, %d] ", rc_x, rc_y, x, y);
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
        //joystick_mix (rc_y, rc_x, &pwm_motor_1, &pwm_motor_2);
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
    //m.motor1_rpm_pcm = buf.motor1_rpm_pcm;
    //m.motor2_rpm_pcm = buf.motor2_rpm_pcm;
    //m.motor3_rpm_pcm = buf.motor3_rpm_pcm;
    //m.motor4_rpm_pcm = buf.motor4_rpm_pcm;
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
