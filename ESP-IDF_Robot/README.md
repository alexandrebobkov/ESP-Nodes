# RC CAR powered by ESP32-C3 Breadboard Adapter (controlled via ESP-NOW)

<img alt="ESP32=C3 RC Car" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/ESP-IDF_Robot/assets/chassi-002.jpg" width="80%"/>

### Designated Pins & GPIOs

__The table below lists GPIOs/Pins programmed to delivery specific operating functions.__

| GPIO | Pin | Assigned Functionality | Notes |
| --- | --- | --- | --- |
| 0 | 16 | Joystick x-axis | ADC1_CH0 |
| 1 | 15 | Joysticj y-axis | ADC1_CH1 |
| 8 | 5 | Joystick push button | |
| 6 | 4 | GPIO controlling clockwise rotation PWM of left motors | LEDC_CHANNEL_1 |
| 5 | 3 | GPIO controlling clockwise rotation PWM of right motors | LEDC_CHANNEL_0 |
| 4 | 2 | GPIO controlling __counter__ clockwise rotation PWM of left motors | LEDC_CHANNEL_2 |
| 7 | 6 | GPIO controlling __counter__ clockwise rotation PWM of right motors | LEDC_CHANNEL_3 |

### Schematic

<img alt="ESP32=C3 RC Car Schematic" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/ESP-IDF_Robot/assets/schematic-002.png" width="80%"/>

## How Does It Work?

### Hardware

### Model Car Firmware

#### Controlling DC Motors

The Model Car uses four DC motors attached to mecanum wheels. The rotation magntutude of each DC motors is controlled by PWM. Each of corresponding PWM value is stored in a struct for later processing. The PWM value can take within a range from 0 to 8091. 

DC motors PWM values are organized in a struct as follows:

```C
struct motors_rpm {
    int motor1_rpm_pcm;
    int motor2_rpm_pcm;
    int motor3_rpm_pcm;
    int motor4_rpm_pcm;
};
```

#### Calculating PWM Values

The DC motors require digital PWM signals which, in turn, depend on analog signals supplied by the joystick stick. Consequently, analog values need to be interpreted into digital PWM signals. ESP32-C3 is capable of sampling voltage (analog signal) for the purpose of forming PWM signal.

As joystick x- and y- coordinates change, so do voltages on corresponding joystick analog outputs. Based on the hardware implementation, the voltage on x- and y- analog outputs can change witin a range from 0V to 3.3V . When ESP32-C3 takes sample of voltages on these outputs, ESP32-C ADC produces values witin a range from 0 to 2048. When joystick is in neutral position, the x- and y- values are (1024, 1024).

Once analog signals are measured, their magnitudes are converted into PWM values; in addition, PWM values __cannot be nagative__. When joystick is in neutral position, the PWM values for all four motors is 0. However, when joystick is moved Front, Back, Left or Right, the PWM values for corresponding motor(s) is(are) updated, and can take value up to 8091.

### Receiver & Controller (ESP-NOW) Firmware

ESP-NOW is used to communicate data between Controller and Receiver.

#### RC Controller

RC Controller uses the two ADC on ESP32-C3 to sample voltage levels on joystick x- and y- axis potentionometers. Then, these values are stored in a struct.

Sensors values are organized in a struct as follows: 

``` C
// Struct holding sensors values
typedef struct {
    uint16_t    crc;                // CRC16 value of ESPNOW data
    uint8_t     x_axis;             // Joystick x-position
    uint8_t     y_axis;             // Joystick y-position
    bool        nav_bttn;           // Joystick push button
} __attribute__((packed)) sensors_data_t;
```

The function _sendData()_ is a core function that sends data to the received using ESP-NOW.

```C
// Function to send data to the receiver
void sendData (void) {
    sensors_data_t buffer;              // Declare data struct

    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;

    // Display brief summary of data being sent.
    ESP_LOGI(TAG, "Joystick (x,y) position ( 0x%04X, 0x%04X )", (uint8_t)buffer.x_axis, (uint8_t)buffer.y_axis);  
    ESP_LOGI(TAG, "pcm 1, pcm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor1_rpm_pcm, (uint8_t)buffer.motor2_rpm_pcm);
    ESP_LOGI(TAG, "pcm 3, pcm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.motor3_rpm_pcm, (uint8_t)buffer.motor4_rpm_pcm);

    // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
    uint8_t result = esp_now_send(receiver_mac, &buffer, sizeof(buffer));

    // If status is NOT OK, display error message and error code (in hexadecimal).
    if (result != 0) {
        ESP_LOGE("ESP-NOW", "Error sending data! Error code: 0x%04X", result);
        deletePeer();
    }
    else
        ESP_LOGW("ESP-NOW", "Data was sent.");
}
```

Since ESP-NOW uses wireless module, Wi-Fi needs to be initialized before configuring ESP-NOW.

```C
/* WiFi is required to run ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );      // Keep configurations in RAM
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));             // Do not change WiFi device mode
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));   // Both sender & receiver must be on the same channel
}
```
The main function contains lines of code that initialize wireless, ESP-NOW, specify configuration variables, and start recurring task.

```C
#include "esp_wifi.h"

void app_main(void)
{
    // Initialize NVS to store Wi-Fi configurations
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // ESP-NOW
    wifi_init();                                    // Initialize Wi-Fi
    esp_now_init();                                 // Call ESP-NOW initialization function
    esp_now_register_recv_cb(onDataReceived);       // Define call back for the event when data is being received
    esp_now_register_send_cb(onDataSent);           // Define call back for the event when data is sent received

    // Set ESP-NOW receiver peer configuration values
    memcpy (peerInfo.peer_addr, receiver_mac, 6);   // Copy receiver MAC address
    peerInfo.channel = 1;                           // Define communication channel
    peerInfo.encrypt = false;                       // Keep data unencrypted
    esp_now_add_peer(&peerInfo);                    // Add peer to 
    xTaskCreate (rc_send_data_task, "RC", 2048, NULL, 15, NULL);
}
```

The _onDataReceived()_ and _onDataSent()_ are two call-bacl functions that get evoked on each corresponding event.

```C
// Call-back for the event when data is being received
void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

    buf = (sensors_data_t*)data;                            // Allocate memory for buffer to store data being received
    ESP_LOGW(TAG, "Data was received");
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
    ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
    ESP_LOGI(TAG, "PCM 1: 0x%04x", buf->motor1_rpm_pcm);
}

// Call-back for the event when data is being sent
void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGW(TAG, "Packet send status: 0x%04X", status);
}
```

The _rc_send_data_task()_ function runs every second to send data over ESP-NOW.

```C
// Continous, periodic task that sends data.
static void rc_send_data_task (void *arg) {

    while (true) {
        if (esp_now_is_peer_exist(receiver_mac))
            sendData();
        vTaskDelay (1000 / portTICK_PERIOD_MS);
    }
}
```

### Variables

| Variable | Value | Description |
| --- | --- | --- |
| MTR_FREQUENCY | 5000 | Default PWM frequency. |

