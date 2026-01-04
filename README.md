# ESP-Nodes
Eco-system of ESP32s Nodes and ATtiny85 peripherals with different functionalities.

<p>The ESP32-Node is designed to be a low-cost, vanila solution for experimenting with embedded and IoT devices. The ESP32-Node is intended to College students taking Electronics program or advanced users who are looking for functional ESP32 embedded board with minimal size.</p>

<img alt="ESP32-Node PCB" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-Node-001.jpg" width="50%"/>

<p><i>ESP32-Node Features:</i></p>

- Uses bare-bones ESP32-WROOM-32D Module [^1]
- Direct connections to all GPIOs
- Miniature in size; only 35.7mm x 35.7mm
- Built-in 3V3 voltage regulator (V<sub>max</sub>=15V) and reversed supply voltage polarity protection
- Pre-wired strapping pins for ensuring proper booting on power-on
- blue power-on LED
- two programmable LEDs
- programmable via [UART](https://github.com/alexandrebobkov/ESP-Nodes/tree/main?tab=readme-ov-file#uart) -> no need for serial drivers

### ESP32-WROOM-32D Module Adapter Pinouts
<p>ESP32-WROOM-32D modules are well suited for Wi-Fi and Bluetooth/Bluetooth LE-based connectivity applications and provide a solid dual-core performance. These modules target a wide variety of applications, ranging from low-power sensor networks to the most demanding tasks, such as voice encoding, music streaming and MP3 decoding.
</p>
<p>The adapter allows interchangability of ESP32 modules between different nodes. The adapter contains minimal components on its PCB, just enough to ensure module's operation. (ESP32 8-N-1)</p>

[ESP32-WROOM Schematic Checklist](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html)

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

### ESP32-C3 Module Adapter Pinouts
ESP32-C3 is a cost-effective, RISC-V-based MCU with Wi-Fi and Bluetooth 5 (LE) connectivity for secure IoT applications. ESP32-C3 Module offfers a cost-effective RISC-V MCU with Wi-Fi and Bluetooth 5 (LE) connectivity for secure IoT applications. [^5]

<p>These modules have a rich set of peripherals and high performance make the two modules an ideal choice for smart homes, industrial automation, health care, consumer electronics, etc</p>
<p>ESP32-C3 has 32-bit RISC-V-based MCU single-core processor with 400KB of SRAM, which is capable of running at 160MHz. t has integrated 2.4 GHz Wi-Fi and Bluetooth 5 (LE) with a long-range support. It has 22 programmable GPIOs with support for ADC, SPI, UART, I2C, I2S, RMT, TWAI, and PWM.</p>

[ESP32-C3 Schematics Checklist](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/schematic-checklist.html)

<img alt="ESP32-C3 Block Diagram" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-C3-diagram.png" width="400px"/>

### Strapping & Control Pins
| Pin | Default | Description |
| --- | --- | --- |
| `EN` | - | H: enables chip <br/> L: disables chip |
| `GPIO2` | n/a | 1 -> Download boot |
| `GPIO8` | n/a | 1 -> Download boot |


### DC Characteristics
| Variable | Parameter | min | Typ | max | Unit |
| --- | --- | --- | --- | --- | :---: |
| V<sub>IH</sub> | High-level Input voltage | 0.75xV<sub>DD</sub> | - | V<sub>DD</sub> + 0.3 | V |
| I<sub>IH</sub> | High-level Input current | - | - | 50 | nA |
|||||||
| V<sub>IL/sub> | Low-level Input voltage | -0.3 | - | 0.25xV<sub>DD</sub> | V |
| I<sub>IL</sub> | Low-level Input current | - | - | 50 | nA |
|||||||
| V<sub>OH</sub> | High-level Output voltage | 0.8xV<sub>DD</sub> | - | - | V |
| I<sub>OH</sub> | High-level Source current | - | 40 | - | mA |
|||||||
| V<sub>OL/sub> | Low-level Output voltage | - | - | 0.1xV<sub>DD</sub> | V |
| I<sub>OL</sub> | Low-level Sinc current | - | 28 | - | mA |
|||||||
| R<sub>PU</sub> | Pull-up resistor | - | 45 | - | kOm |
| R<sub>PD</sub> | Pull-down resistor | - | 45 | - | kOm |
| V<sub>IH_nRST</sub> | Chip reset release voltage | 0.75xV<sub>DD</sub> | - | V<sub>DD</sub> + 0.3 | V |
| V<sub>IL_nRST</sub> | Chip reset voltage | 0.75xV<sub>DD</sub> | -0.3 | 0.25xV<sub>DD</sub> | V |

### Programming via UART

ESP32 modules can be programmed using USB-UART adapter connected to the corresponding UART pins on the ESP32 module. This approach allows to save space on PCB boards, which is very helpful when PCB dimentions have constraints.

> [!IMPORTANT]
> Remember to swap Tx and Rx between receiver and sender (i.e. Rx on a sender side becomes Tx on a receiver side).

Connect Tx and Rx to their sorresponding pins: GPIOs `GPIO03` and `GPIO01` on ESP32-Module; or pins `34` and `35` on ESP32 adapter board.

<img alt="ESP32 Programming via UART" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/UART.jpg" width="400px"/>

## Unified Configuration Framework

ESP Nodes performing specific operations can use different ESP32 Modules for optimization purposes. As dicersity of ESP32s being used by the Nodes increases, so does the code. However, in order to keep core code independent from the ESP32 Module being used, the so-called configurations specific for the particular ESP32 Module are defined in config.h file.

> [!IMPORTANT]
> Remember to add entry for congif.h inside .gitignore file to prevent Git from pushing security sensitive data to the repository.

```C
const *char WIFI_SSID = "IoT_bots";
const *char WIFI_PASSWORD = "405405405";

/* ESP32-WROOM Module */
#ifdef ESP32-WROOM
#define SCL_PIN         (22)
#define SDA_PIN         (21)
#endif

/* ESP32-C3 Module */

/* ESP32-S3 Module */

/* ESP32-C3 Super Mini */
#ifdef ESP32-C3-SuperMini
#define SCL_PIN         (7)
#define SDA_PIN         (6)
#define SYS_LED_PIN     (8)
#define SCK_PIN         (8)
#define MISO_PIN        (9)
#define MOSI_PIN        (10)
#define UART_RX_PIN     (20)
#define UART_TX_PIN     (21)
#define ADC1_PIN        (1)
#define ADC2_PIN        (0)
#define A0_PIN          (2)
#define A1_PIN          (3)
#define A2_PIN          (4)
#define A3_PIN          (5)
#endif

/* BME-280 */
#ifdef BME280
#define I2C_ADDRESS_BME280  0x76
#endif
/* BMP-280 */
#ifdef BME280
#define I2C_ADDRESS_BMP280  0x76
#endif

```

Bare-bones ESP32 module can be programmed via UART interface (`GPIO03` and `GPIO01`) using USB to UART adapter.

## ESP-IDF Framework

_Configuration menu custom variables_

_Kconfig.projbuild_ file contains custom menu configuration for setting user-defined variables such as button GPIO, device description, etc.

``` text
menu "Example Configuration"

    config EXAMPLE_BOARD_BUTTON_GPIO
        int "Boot Button GPIO"
        default 9 if IDF_TARGET_ESP32C3 || IDF_TARGET_ESP32C6
        default 0
        help
            GPIO number on which the "Boot" button is connected. This is generally used
            by the application for custom operations like toggling states, resetting to defaults, etc.

    config EXAMPLE_ENABLE_TEST_NOTIFICATIONS
        bool "Test Notifications"
        default n
        help
            Enable this option to test mobile push notifications. When enabled, turning on the switch using
            push button will trigger a parameter notification {"Switch":{"Power":true}} and turning off will
            trigger an alert "Switch was turned off".

    config EXAMPLE_OUTPUT_GPIO
        int "Output GPIO"
        default 19
        help
            This is an output GPIO that will be connected to a relay or other driver circuit in most cases.
            If the power changes, this GPIO output level will also change.

    config EXAMPLE_DEVICE_DESCRIPTION
        int "IoT Node description"
        default "Lamp"
        help
            This is the description of IoT Node; appears on ESP RainMaker dashboard device detail.
endmenu
```

### Frequently Used Commands

_To write firmware to the ESP's flash chip:_

```
esptool.py --port COM_ --chip esp32 write_flash 0x1000 firmware-name.bin
```

_To verify firmware:_
```
esptool.py verify_flash --diff yes firmware-name.bin
```

_To read firmware from the ESP's flash chip:_
```
esptool.py --port COM_ -b --chip esp32 460800 read_flash 0 ALL flash-contents.bin
```

_To erase ESP32 flash chip:_
```
esptool.py --port COM_ --chip esp32 erase_flash
```

_To read built-in MAC address:_
```
esptool.py --port COM_ --chip esp32 read_mac
```

_To convert ELF to binary:_
```
esptool.py --chip esp32 elf2image firmware.elf
```

_To output .bin image details:_
```
esptool.py image_info --version 2 firmware-name.bin
```

_To read RAM:_
```
esptool.py read_mem 0x400C0000
```

_To read flash chip registers:_
```
esptool.py read_flash_status --bytes 2
```

In order for OTA to push new firmware, update version number saved in the `PROJECT_VER` filed in _CMakeList.txt_ file.

```text
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if(DEFINED ENV{RMAKER_PATH})
  set(RMAKER_PATH $ENV{RMAKER_PATH})
else()
  set(RMAKER_PATH ${CMAKE_CURRENT_LIST_DIR}/../..)
endif(DEFINED ENV{RMAKER_PATH})

# Add RainMaker components and other common application components
set(EXTRA_COMPONENT_DIRS ${RMAKER_PATH}/components/esp-insights/components ${RMAKER_PATH}/components ${RMAKER_PATH}/examples/common)

set(PROJECT_VER "1.2.1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-C3_Table-Lamp)
```

## Temperature Node. The Key Elements and Components

The **Temperature Node** broadcasts the air temperature, atmospheric pressure and air humidity over secured MQTT.

### I2C Air Temperature Thermostat (LM75A)

The LM75A is an industry-standard digital temperature sensor. The LM75A provides 9-bit digital temperature readings with an accuracy of ±2°C from –25°C to 100°C
and ±3°C over –55°C to 125°C. The LM75A operates with a single supply from +2.7 V to +5.5 V. Communication is accomplished over a 2-wire interface which operates up to 400kHz.
The LM75A has three address pins, allowing up to eight LM75A devices to operate on the same 2-wire bus. The LM75A has a dedicated over-temperature output (O.S.) with
programmable limit and hysteresis. This output has programmable fault tolerance, which allows the user to
define the number of consecutive error conditions that must occur before O.S. is activated.


```text
address: 0x49
```

### I2C Air Temperature, Pressure and Humidity Sensor Board (BME280)

BME280 is combined temperature, humidity and pressure sensor. The unit combines high linearity and high accuracy sensors and is perfectly feasible for low current consumption, long-term stability and high EMC robustness. The humidity sensor offers an extremely fast response time and therefore supports performance requirements for emerging applications such as context awareness, and high accuracy over a wide temperature range.[^2]

Below is the functional diagram of BME-/BMP-280. Notable difference between the two devices, is that BME-280 is capable of measuring relative humidity. BME-280 has square shape, while BMP-280 has rectangular shape.

<img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/bme280_functional-diagram.png" width="400px"/>

BME-280 can come in a ready-to-use PCB ...

<img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/BME280.jpg" width="200px"/>

> [!IMPORTANT]
> BM**E**280[^2] and BM**P**280[^3] look almost identical. However, BM**E**280 sensor has a square form (and can measure air temperature, pressure and humidity), while BM**P**280 has a rectangular form (and can only measure air temparature and pressure). In addition, the two sensor boards can have different I<sup>2</sup>C IDs (addresses). Note, that the commands (sensor registers addresses) are interchangable between BME280 and BMP280.
>
> <img alt="ESP32-Node Pinout" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/BME280-BMP280.jpg" width="100px"/>

**4-pin variant**

The BME280 sensor board interface uses 4 pins and is 13mm by 10.5mm in size. The four pins are `VIN`, `GND`, `SCL` and `SDA`. The measured values are sent via I<sup>2</sup>C protocol. The I<sup>2</sup>C slave address is pre-defined and can take value either 0x76 or 0x77 (BME280 Datasheet, page 32)[^4].

### Wiring

| Pin | ESP32 GPIO |
| --- | --- |
| `SCL` | `GPIO22` |
| `SDA ` | `GPIO21` |

### Reading Values

```C
i2c_master_write_read_device(I2C_MASTER_NUM, 0x76, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
```

<p>Data readout is done by starting a burst read from 0xF7 to 0xFC (temperature and pressure) or from 0xF7 to 0xFE (temperature, pressure, and humidity). The data are rad out in an unsigned 20-bit format both for pressure and for temperature, and in an unsigned 26-bit format for humidity. After the uncompensated values for pressure, temperature, and humidity have been read, the actual humidity, pressure and temperature needs to be calculated using the compensation parameters stored in the device.</p>

<img alt="BME-/BMP-280 Memory Map" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/BME280_BMP280_Registers.png" width="90%"/>

BME-/BMP-280 can communicate via I<sup>2</sup>C. The two diagrams below summarize algorithm of reading and writting values to/from the sensor.

<img alt="BME-/BMP-280 I2C Read & Write" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/bme280_i2c_read-write.png" width="100%"/>

## Display Node

```text
OLED address = 0x3C
0.91" 128x32
```

### e-Paper
#### ESP32-WROOM Module

Wiring table

| ESP32-WROOM | e-Paper |
| --- | --- |
| `GPIO4` (hSPI HD) | `BUSY` (Purple) |
| `GPIO5` (vSPI CS) | `CS` (Orange) |
| `GPIO16` (RxD) | `RST` (White) |
| `GPIO17` (TxD) | `DC` (Green) |
| `GPIO18` (vSPI CLK) | `CLK` (Yellow) |
| `GPIO23` (vSPI D) | `DIN` (Blue) |

#### ESP32-C3

| ESP32-C3 | e-Paper |
| --- | --- |
| `GPIO7` (fSPI D) | `DIN` (Blue) |
| `GPIO10` (fSPI CS) | `CS` (Orange) |
| `RXD` `GPIO20` (RxD) | `RST` (White) |
| `TXD` `GPIO21` (TxD) | `DC` (Green) |
| `GPIO6` (fSPI CLK) | `CLK` (Yellow) |
| `GPIO4` (fSPI HD) | `BUSY` (Purple) |

# ESP32-C3 Module and Breadboard Power Supply Combo

_ESP32-C3 Module and breadboard power supply combo board saves precious space on a breadboard giving you more room for placing components._

> [!IMPORTANT]
> Combo board can supply 3.3V when powered by USB-C port. In order for the combo board to supply 5V, connect external power supply to the coressponding power pins.

<img alt="BME-/BMP-280 I2C Read & Write" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-C3-BreadBoardAdapter-001.jpg" width="100%"/>

# ESP32-C3 Temperature Sensor Relay Board

<img alt="BME-/BMP-280 I2C Read & Write" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/ESP32-C3-WROOM-Node-Temperature.png" width="100%"/>

_ESP32-C3 SuperMini + Temperature I<sup>2</sup>C Sensor + Power Relay_

# Espressif ESP RainMaker and Matter Provisioning

ESP RainMaker is a complete, yet light-weight, AIoT solution that enables private Cloud deployment for your business in a simple, cost-effective and efficient manner. ESP RainMaker is a light-weight AIoT Cloud software, fully integrated into the AWS serverless architecture, which allows customers to build, develop and deploy customized AIoT solutions with a minimum amount of code and maximum security. [^7]

Espressif offers a complete solution that includes private deployment of IoT cloud, mobile apps, voice assistant skills, and product solution support. This allows customers to build their very own brand of an IoT business in as efficient, affordable and speedy as possible a way.

Matter [^9] [^10] is an industry-unifying standard that provides reliable and secure connectivity for smart-home devices. It is an IP-based connectivity protocol that works on Wi-Fi, Ethernet, and Thread (over 802.15.4 radio) transports with Bluetooth LE being used for commissioning. The Matter standard is defined by the Connectivity Standards Alliance in association with all the industry leaders.

[ESP**RAINMAKER** dashboard](https://dashboard.rainmaker.espressif.com/sign-up)

[ESP Rainmaker Tutorial](https://github.com/alexandrebobkov/ESP-Nodes/blob/main/Tutorial-Rainmaker.md)

Standard set of IoTs types include: [^8] [^11]

- Lightbulb
- Switch
- Light
- Fan
- Temperature Sensor
- Outlet
- Plug
- Socket
- Lock
- Internal Blinds
- External Blinds
- Garage Door
- Speaker
- Air Conditioner
- Thermostat
- TV
- Washer
- Contact Sensor
- Motion Sensor
- Door Bell
- Security Panel
- Water Heater
- Other


---
## REFERENCES

[^2]: https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
[^3]: https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/
[^4]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
[^5]: https://www.espressif.com/en/news/ESP32_C3
[^6]: https://dashboard.rainmaker.espressif.com/login
[^7]: https://rainmaker.espressif.com/
[^8]: https://rainmaker.espressif.com/docs/get-started/
[^9]: https://www.espressif.com/en/solutions/device-connectivity/esp-matter-solution
[^10]: https://github.com/project-chip/connectedhomeip
[^11]: https://rainmaker.espressif.com/docs/standard-types
