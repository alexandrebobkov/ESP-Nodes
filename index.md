---
layout: default
---

# ESP-Nodes
Eco-system of ESP32s Nodes and ATtiny85 peripherals with different functionalities.

<p>The ESP32-Node is designed to be a low-cost, vanila solution for experimenting with embedded and IoT devices. The ESP32-Node is intended to College students taking Electronics program or advanced users who are looking for functional ESP32 embedded board with minimal size.</p>

<img alt="ESP32-Node PCB" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/ESP32-Node-001.jpg" width="50%"/>

<p><i>ESP32-Node Features:</i></p>

- Uses bare-bones ESP32-WROOM-32D Module [^1]
- Direct connections to all GPIOs
- Miniature in size; only 35.7mm x 35.7mm
- Built-in 3V3 voltage regulator (V<sub>max</sub>=15V) and reversed supply voltage polarity protection
- Pre-wired strapping pins for ensuring proper booting on power-on
- blue power-on LED
- two programmable LEDs
- programmable via [UART](https://github.com/alexandrebobkov/ESP-Nodes/tree/main?tab=readme-ov-file#uart) -> no need for serial drivers

# ESP32-C3 DevBoard Power Supply Combo

<img alt="ESP32-Node PCB" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/20240813_214109.jpg" width="70%"/>

# Rainmaker
[Rainmaker Tutorial](./rainmaker.html).

----
## REFERENCES
[^1]: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf
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
