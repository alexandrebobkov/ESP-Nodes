#ifndef CONFIG_H
#define CONFIG_H

/*
    Configuration variables
*/

/*  ============================
    Joystick
    ============================ 
*/
#define PROJ_X                      (1)                     // ADC1_CH1; 0 GPIO joystick, x-axis
#define PROJ_Y                      (0)                     // ADC1_CH0; 1 GPIO joystick, y-axis
#define NAV_BTN                     (8)                     // 8 GPIO joystick button

/*  ============================
            ESP NOW
    ============================

    ESP32-C3 Luatos ESP32C3 board MAC:      54:32:04:46:71:80
    ESP32-C3 SuperMini MAC:                 34:b7:da:f9:33:8d
    ESP32-C3 Breadboard MAC:                e4:b0:63:17:9e:45
*/
#endif