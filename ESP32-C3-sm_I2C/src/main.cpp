#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
//#include "epaper.h"

Adafruit_BME280 bme;

// Define tasks.
TaskHandle_t TaskStatusLED, TaskSensorValues;

// Dummy task blinking built-in LED
void TaskStatusLEDCode (void* parameters) {
  Serial.print("Task 0 running on core # ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    digitalWrite(SYS_LED_PIN, LOW);
    vTaskDelay(250 / portTICK_RATE_MS);
    digitalWrite(SYS_LED_PIN, HIGH);
    vTaskDelay(250 / portTICK_RATE_MS);
    digitalWrite(SYS_LED_PIN, LOW);
    vTaskDelay(250 / portTICK_RATE_MS);
    digitalWrite(SYS_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }                
}

void TaskSensorValuesCode (void* parameters) {

  for (;;) {
    Serial.println("BME-280 Sensors Readings ...");
    Serial.print("Temperature:\t\t");
    Serial.print(bme.readTemperature());
    Serial.println("\t°C");
    Serial.print("Humidity:\t\t");
    Serial.print(bme.readHumidity());
    Serial.println("\t%");
    Serial.print("Barometric Pressure:\t");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println("\tkPa");
    // ADC GPIO1
    light_sensor_reading = analogRead(ADC1);
    Serial.print("Light:\t\t\t");
    Serial.println(light_sensor_reading);
    Serial.println("");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }  
}

void setup() {
  Serial.println("Please wait 2   seconds ...");
  delay(2000);
  Serial.begin(9600);
  Serial.println();
  
  Serial.println("Setting up GPIOs ...");

  pinMode(LED_PIN, OUTPUT);

  Serial.println("GPIO setup done");  

  Serial.println("Setting up BME280 sensor");
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  //unsigned status = bme.begin(0x76); // 0x76
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    int n = 10;
    while (1) { //n > 0) {
      digitalWrite(LED_PIN, LOW);
      delay(150);
      digitalWrite(LED_PIN, HIGH);
      delay(150);
      Serial.println("Could not find a valid BME/BMP280 sensor!");
      n--;
    }
  }
  else {
    Serial.println("Sensor was found.");
    xTaskCreate(TaskStatusLEDCode, "Status LED Task", 4096, NULL, 10, &TaskStatusLED);
    xTaskCreate(TaskSensorValuesCode, "Status LED Task", 4096, NULL, 5, &TaskSensorValues);
  }

  /*// Initialize ePaper display
  display.init(115200);
  display.fillScreen(GxEPD_BLACK);
  display.update();
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMono9pt7b);
  //display.setCursor(205, 10);
  //display.print("ESP32_DisplayNode"); // Takes 195 pixels in width

  display.setCursor(5, 5);
  display.setFont(&TomThumb);
  display.print("10.100.50.xxx");
  display.update();*/
}

void loop() {

  /*Serial.println("Loop");
  Serial.print("SS = ");
  Serial.println(SS);
  Serial.print("MOSI = ");
  Serial.println(MOSI);
  Serial.print("MISO = ");
  Serial.println(MISO);
  Serial.print("SCK = ");
  Serial.println(SCK);
  Serial.print("BUSY = ");
  Serial.println(BUSY_PIN);
  sleep(1);*/

}

/*
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32C3)
#define DF_GFX_SCK 4
#define DF_GFX_MOSI 6
#define DF_GFX_MISO GFX_NOT_DEFINED
#define DF_GFX_CS 7
#define DF_GFX_DC 2
#define DF_GFX_RST 1
#define DF_GFX_BL 3*/