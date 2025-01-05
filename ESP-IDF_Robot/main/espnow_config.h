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