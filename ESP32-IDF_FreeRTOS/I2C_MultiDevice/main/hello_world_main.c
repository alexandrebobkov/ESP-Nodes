/*
 * freeRTOS simultaneous tasks.
 * Created on: Aug 5, 2025
 * Updated on:
 * 
 * By: Alexander Bobkov
 * 
 * ESP-IDF version: 5.4.1
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

SemaphoreHandle_t xMutex;
QueueHandle_t xQueue1;
QueueHandle_t xQueue2;

typedef struct {
    uint32_t num1;
    uint32_t num2;
    uint32_t num3;
} SensorsData;

static int cntdn;

void task1(void *pvParameters);
void task2(void *pvParameters);
void display_task(void *pvParameters);
void restart_task(void *pvParameters);

void app_main(void)
{
    cntdn = 10;
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("ESP Module ID: %d with %d CPU core(s),\n%s%s%s%s,\n",
           chip_info.model,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, \n", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());



    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        printf("Failed to create mutex\n");
        return;
    }

    xQueue1 = xQueueCreate(10, sizeof(SensorsData));
    xQueue2 = xQueueCreate(10, sizeof(SensorsData));
    if (xQueue1 == NULL || xQueue2 == NULL) {
        printf("Failed to create queues\n");
        return;
    }

    xTaskCreate(task1, "Task1", 2048, NULL, 5, NULL);
    xTaskCreate(task2, "Task2", 2048, NULL, 5, NULL);
    xTaskCreate(restart_task, "TaskRestart", 2048, NULL, 20, NULL);

}


void task1(void *pvParameters) {
    int x = 0;
    
    while (1) {
        if (xSemaphoreTake(xMutex, 1500)) {
            printf("Task 1 is running\n");
            printf("This is Task #1\n\n");
            vTaskDelay((500)); // Delay for 1 second
            xSemaphoreGive(xMutex);
        }
        else {
            printf("Task 1 timed out waiting for mutex\n");
        }
        vTaskDelay((100));
    }
}

void task2(void *pvParameters) {
    int y = 0;

    while (1) {
        if (xSemaphoreTake(xMutex, 1500)) {
            printf("Task 2 is running\n");

            SensorsData data = {
                .num1 = 0,
                .num2 = y,
                .num3 = 0,
            };
            xQueueSend(xQueue1, &data, 0);
            printf("Task 1 sent data");
            y++;

            printf("This is Task #2\n\n");
            vTaskDelay((2000)); // Delay for 2 seconds
            xSemaphoreGive(xMutex);
        }
        else {
            printf("Task 2 timed out waiting for mutex\n");
        }
        vTaskDelay((100));
    }
}

void restart_task(void *pvParameters) {
    printf("Restarting system in %d seconds...\n", cntdn);
    while (1) {
        vTaskDelay(cntdn * 1000); // Delay for 10 seconds
        esp_restart();
    }
}


void display_task(void *pvParameters) {
    TaskData data;
    while (1) {
        if (xQueueReceive(xQueue1, &data, pdMS_TO_TICKS(500))) {
            printf("Display Task received from Task 1: id=%d, message=%s\n", data.id, data.message);
        }
        if (xQueueReceive(xQueue2, &data, pdMS_TO_TICKS(500))) {
            printf("Display Task received from Task 2: id=%d, message=%s\n", data.id, data.message);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


