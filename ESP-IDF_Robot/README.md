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
    int motor1_gpio;
    int motor2_rpm_pcm;
    int motor2_gpio;
    int motor3_rpm_pcm;
    int motor3_gpio;
    int motor4_rpm_pcm;
    int motor4_gpio;
};
```

#### Calculating PWM Values

The DC motors require digital PWM signals which, in turn, depend on analog signals supplied by the joystick stick. Consequently, analog values need to be interpreted into digital PWM signals. ESP32-C3 is capable of sampling voltage (analog signal) for the purpose of forming PWM signal.

As joystick x- and y- coordinates change, so do voltages on corresponding joystick analog outputs. Based on the hardware implementation, the voltage on x- and y- analog outputs can change witin a range from 0V to 3.3V . When ESP32-C3 takes sample of voltages on these outputs, ESP32-C ADC produces values witin a range from 0 to 2048.

### Receiver & Controller (ESP-NOW) Firmware

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

### Variables

| Variable | Value | Description |
| MTR_FREQUENCY | 5000 | Default PWM frequency. |
