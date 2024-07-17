# ESP RAINMAKER TUTORIAL

This tutorial walks you through the most important steps neccessary to provision ESP32-WROOM device.

> [!TIP]
> **WHAT'S REQUIRED:**
>
> ESP32-WROOM, either a Module or DevKit</br>
> Wi-Fi Access Point, preferrably with dedicated vLAN for IoT devices</br>
> Smart phone with RainMaker installed

### ESP-IDF: Create a New Project

Launch ESP-IDF extention on VS Code

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_001.png" width="70%"/>

Specify project name, directory path and ESP32 module (ESP32-WROOM or ESP32-C3). In addition, you may state the Serial port where ESP32 Module is connected to; this setting can be changed latter, if neccessary.

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_002.png" width="70%"/>

On the next screen, specify ESP Rainmaker extention and select example code to start with. For this tutorial, an example code for _ _Switch_ _ was selected. Click "Create project using template switch".

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_003.png" width="70%"/>

### Modify Template Code

Make the following changes to app_main.cpp source file.

Change the TAG variable as follows:

```C++
static const char *TAG = "ESP32-Nodes app main";
```

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_004.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_005.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_006.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_007.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_008.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_009.png" width="70%"/>

<img alt="ESP-IDF. Create a New Project" src="https://github.com/alexandrebobkov/ESP-Nodes/blob/main/assets/Espressif-Rainmaker_010.png" width="70%"/>