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
| `4` `5` `6` `7` | **Inputs Only** `GPIO36` `GPIO39` `GPIO34` `GPIO35` | ADC1_CH0, ADC1_CH3, ADC1_CH6, ADC1_CH7 |
| `8` `9` | `GPIO32` `GPIO33` | TOUCH_9, TOUCH_8 |
| `10` `11` | `GPIO25` `GPIO26` | DAC_1, DAC_2 |
| `12` | `GPIO27` | TOUCH_7 |
| `13` `14` | `GPIO14` `GPIO12` | `HSPI_CLK` `HSPI_MISO` TOUCH_6 TOUCH_7 |
| `16` | `GPIO13` | `HSPI_MOSI` ADC2_CH4 |
| `17` `18` `19` `20` `21` `22` | **Internal Use** `GPIO09` `GPIO10` `GPIO11` `GPIO08` `GPIO07` `GPIO06` |
| `23` `24` | `GPIO02` `GPIO15` | AD2_CH3 ADC2_CH2 |
| `25` `26` | `GPIO00` `GPIO04` | TOUCH_1 TOUCH_0 |
| `27` `28` | `GPIO16` `GPIO17` | `UART2_RXD` `UART2_TXD` |
| `29` `30` `31` | `GPIO05` `GPIO18` `GPIO19` | `VSPI_CS0` `VSPI_CLK` `VSPI_MISO` |
| `32` | Not Connected |
| `33` | `GPIO21` | `SDA` |
| `34` `35` | `GPIO03` `GPIO01` | UART0_RXD UART0_TXD |
| `36` | `GPIO22` | `SCL` |
| `37` | `GPIO23` | VSPI_MOSI | 