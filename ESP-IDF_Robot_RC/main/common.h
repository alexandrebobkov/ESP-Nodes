/*
    ESPNOW Module common for both controller and receiver.
*/

#ifndef COMMON_H
#define COMMON_H

typedef struct {
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
} __attribute__((packed)) data_t;

typedef struct {
    int len;                                    // Length of ESPNOW data to be sent, unit: byte.
    uint8_t     *buffer;                        // Buffer; pointer to the data struct.
    uint8_t     dest_mac[ESP_NOW_ETH_ALEN];     // MAC address of destination device.
} espnow_data_encapsulation_t;

typedef struct {
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
} __attribute__((packed)) sensors_data_t;

typedef struct {
    int len;                                // Length of ESPNOW data to be sent, unit: byte.
    uint8_t     *buffer;                      // Buffer; pointer to the data struct.
    uint8_t     dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} espnow_data_packet_t;

static void wifi_init(void);