---
layout: tutorial
---

# ESP RAINMAKER TUTORIAL

This tutorial walks you through the most important steps required for provisioning device powered by ESP32-WROOM, ESP32-S2 or ESP32-C3 module.

__Minimum Requirements:__ </br>
ESP32-WROOM, either a Module or DevKit</br>
Wi-Fi Access Point, preferrably with dedicated vLAN for IoT devices</br>
Smart phone with RainMaker installed

### I. ESP-IDF: Create a New Project

The first step is to create a new project with ESP-IDF Rainmaker extention using Visual Studio Code.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_001.png" width="70%"/>

Specify the project name, directory path and ESP32 module that will be used (i.e. ESP32-WROOM, ESP32-S2 or ESP32-C3). In addition, you may specify the Serial port where ESP32 Module is connected to; the serial port can be changed latter, if neccessary.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_002.png" width="70%"/>

On the next screen, select the ESP Rainmaker extention and choose the example code to start with. For this tutorial, an example code for __Switch__ was selected. Click "Create project using template switch".

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_003.png" width="70%"/>

### II. Modify the Template Code

Once project is created, make the following changes to the **app_main.cpp** source file.

Change the TAG variable to reflect your device name as follows:

```C++
static const char *TAG = "ESP32-Nodes app main";
```

```C++
esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Switch");
```

```C++
switch_device = esp_rmaker_device_create("ESP32-Nodes Switch", ESP_RMAKER_DEVICE_SWITCH, NULL);
```

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_004.png" width="70%"/>

### III. Modify Configuration

ESP-IDF **menuconfig** contains settings that define hardware configuration. For example, GPIO used to control power relay, etc. To set configuration values, navigate to the ESP-IDF: SDK Configuration Editor (menuconfig). Some of the default values are pre-defined by the RainMaker Switch template; however, the specific number of GPIO used to turn LED Lights on or off needs to be entered.

To do so, click on Example Configuration section, and change _Output GPIO_ value to __4__, which corresponds to the GPIO connected to the LED lights control pin.

Set _Claiming Type_ to __Assisted__, and _Provisioning Transport Method_ to __BLE__.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_005.png" width="70%"/>

Click _Save_ and then build the project by running the command _ESP-IDF: Build Project_.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_006.png" width="70%"/>

Building the project can take some time.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_007.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_008.png" width="70%"/>

### IV. Provisioning RainMaker Device

Once project is successfully built, and ESP32-WROOM module is flashed, a device needs to be connected to the Rainmaker cloud service. To do so, reboot your device and open Serial Monitor in order to access _provisioning QR Code_. At this point you can start adding your device to your RainMaker dashboard. On your smartphone, launch the _ESP RainMaker_ app and click _Add device_. You'll be asked to scan the QR code. If provisioning is successful, _ESP RainMaker_ app will take you to the nest steps.

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_009.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Espressif-Rainmaker_010.png" width="70%"/>

### V. Connecting Rainmaker Device to Alexa

Once device is successfully provisioned, it can be linked to your Alexa account. In order to accomplish this, click on _Link to Alexa_ from Rainmaker app.

<img alt="Alexa Account Linking to RainMaker" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Screenshot_20240716-213521~2.png" width="70%"/>

<img alt="RainMaker Provisioning. Adding Wi-Fi" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Screenshot_20240716-223344~2.png" width="70%"/>

<img alt="RainMaker Provisioning" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Screenshot_20240716-223358~2.png" width="70%"/>

<img alt="RainMaker. ESP32 Node Switch Setting" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Screenshot_20240716-230310~2.png" width="70%"/>

<img alt="RainMaker. ESP32 Node Switch" src="https://alexandrebobkov.github.io/ESP-Nodes/assets/Screenshot_20240716-230318~2.png" width="70%"/>

