# RC CAR powered by ESP32-C3 Breadboard Adapter & conttolled via ESP-NOW

<img alt="ESP32=C3 RC Car" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/ESP-IDF_Robot/assets/chassi-002.jpg" width="80%"/>

## Designated Pins & GPIOs

__The table below lists GPIOs/Pins programmed to delivery specific operating functions.__

| GPIO | Pin | Assigned Functionality | Notes |
| --- | --- | --- | --- |
| 0 | 16 | Joystick x-axis | ADC1_CH0 |
| 1 | 15 | Joysticj y-axis | ADC1_CH1 |
| 8 | 5 | Joystick push button | |
| 6 | 4 | GPIO controlling PWM for the front left motor | LEDC_CHANNEL_1 |
| 5 | 3 | GPIO controlling PWM for the front right motor | LEDC_CHANNEL_0 |

## Variables

| Variable | Value | Description |
| MTR_FREQUENCY | 5000 | Default PWM frequency. |