# ESP-Nodes
Eco-system of ESP32s Nodes and ATtiny85 peripherals with different functionalities.

<p>The ESP32-Node is designed to be a low-cost solution for experimenting with embedded and IoT devices. The ESP32-Node is intended to College students taking Electronics program or advanced users who are looking for functional ESP32 embedded board with minimal size.</p>

<img alt="ESP32-Node PCB" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-Node-001.jpg" width="50%"/>

<p><i>ESP32-Node Features:</i></p>

- ESP32-WROOM-32D Module
- Direct connections to all GPIOs
- Miniature in size, only 35.7mm x 35.7mm
- Built-in 3V3 voltage regulator and reversed voltage protection
- Pre-wired strapping pins for ensuring proper booting on power-on
- blue power-on LED
- two programmable LEDs
- programmable via UART0 -> no need for serial drivers

### Pinouts

<img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-Node-pinout.png" width="350px"/>

| Physical Pin | Descrption and Logical Pin | Extended Function |
| --- | --- | --- |
| `1` `15` `38` | Ground | |
| `3` | EN | |
| `4` `5` `6` `7` | Inputs `GPIO36` `GPIO39` `GPIO34` `GPIO35` | ADC1_CH0, ADC1_CH3, ADC1_CH6, ADC1_CH7 |
| `8` `9` | `GPIO32` `GPIO33` | TOUCH_9, TOUCH_8 |
| `10` `11` | `GPIO25` `GPIO26` | DAC_1, DAC_2 |