# ESP-Nodes
Eco-system of ESP32s Nodes and ATtiny85 peripherals with different functionalities.

<p>The ESP32-Node is designed to be a low-cost, vanila solution for experimenting with embedded and IoT devices. The ESP32-Node is intended to College students taking Electronics program or advanced users who are looking for functional ESP32 embedded board with minimal size.</p>

<img alt="ESP32-Node PCB" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-Node-001.jpg" width="50%"/>

<p><i>ESP32-Node Features:</i></p>

- ESP32-WROOM-32D Module [^1]
- Direct connections to all GPIOs
- Miniature in size, only 35.7mm x 35.7mm
- Built-in 3V3 voltage regulator (15V input max) and reversed supply voltage polarity protection
- Pre-wired strapping pins for ensuring proper booting on power-on
- blue power-on LED
- two programmable LEDs
- programmable via [UART](https://github.com/alexandrebobkov/ESP-Nodes/tree/main?tab=readme-ov-file#uart) -> no need for serial drivers

### ESP32-WROOM-32D Module Adapter Pinouts
<p>The adapter allows interchangability of ESP32 modules between different nodes. The adapter contains minimal components on its PCB, just enough to ensure module's operation.</p>

<img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-Node-pinout.png" width="350px"/>

| Physical Pin | Descrption and Logical Pin | Extended Function |
| --- | --- | --- |
| `1` `15` `38` | Ground | |
| `2` | `3V3` | |
| `3` | EN | |
| `4` `5` `6` `7` | **Inputs Only** `GPIO36` `GPIO39` `GPIO34` `GPIO35` | ADC1_CH0, ADC1_CH3, ADC1_CH6, ADC1_CH7 |
| `8` `9` | `GPIO32` `GPIO33` | TOUCH_9, TOUCH_8 |
| `10` `11` | `GPIO25` `GPIO26` | DAC_1, DAC_2 |
| `12` | `GPIO27` | TOUCH_7 |
| `13` `14` | `GPIO14` `GPIO12` | `HSPI_CLK` `HSPI_MISO` TOUCH_6 TOUCH_7 |
| `16` | `GPIO13` | `HSPI_MOSI` ADC2_CH4 |
| `17` `18` `19` `20` `21` `22` | **Module Internal Use Only** `GPIO09` `GPIO10` `GPIO11` `GPIO08` `GPIO07` `GPIO06` |
| `23` `24` | `GPIO02` `GPIO15` | AD2_CH3 ADC2_CH2 |
| `25` `26` | `GPIO00` `GPIO04` | TOUCH_1 TOUCH_0 |
| `27` `28` | `GPIO16` `GPIO17` | `UART2_RXD` `UART2_TXD` |
| `29` `30` `31` | `GPIO05` `GPIO18` `GPIO19` | `VSPI_CS0` `VSPI_CLK` `VSPI_MISO` |
| `32` | Not Connected |
| `33` | `GPIO21` | `SDA` |
| `34` `35` | `GPIO03` `GPIO01` | UART0_RXD UART0_TXD |
| `36` | `GPIO22` | `SCL` |
| `37` | `GPIO23` | VSPI_MOSI | 

[^1]: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf


### UART

## Temperature Node Key Elements and Components

The **Temperature Node** broadcasts the air temperature, atmospheric pressure and air humidity over secured MQTT.

### I2C Air Temperature, Pressure and Humidity Sensor Board (BME280)

The unit combines high linearity and high accuracy sensors and is perfectly feasible for low current consumption, long-term stability and high EMC robustness. The humidity sensor offers an extremely fast response time and therefore supports performance requirements for emerging applications such as context awareness, and high accuracy over a wide temperature range.[^2]

<img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/BME280.jpg" width="200px"/>

> [!IMPORTANT]
> BM**E**280[^2] and BM**P**280[^3] look almost identical. However, BME280 sensor has a square form, while BMP280 has a rectangular form. In addition, the two sensor boards have different I2C addresses.
>
> <img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/BME280-BMP280.jpg" width="100px"/>

**4-pin variant**

The BME280 sensor board interface uses 4 pins and is 13mm by 10.5mm in size. The four pins are `VIN` `GND` `SCL` and `SDA`. The measured values are sent via I2C protocol.

### Wiring

| Sensor Pin | ESP32 GPIO |
| --- | --- |
| `SCL` | `GPIO22` |
| `SDA ` | `GPIO21` |

[^2]: https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
[^3]: https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/