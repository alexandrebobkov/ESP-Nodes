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
TaskHandle_t TaskStatusLED, TaskSensorValues, TaskLightsAuto;

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

void TaskLightsAutoCode(void* parameters) {
  for (;;) {
    // If lighting is dark, then turn lights ON and increase delay interval
    // Swap HIGH with LOW since we're using NPN transistor to control relay.
    if (light_sensor_reading > 1500) {
      digitalWrite(LIGHTS_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
    // If lighting is bright, then turn lights OFF and decrease delay interval
    else {
      digitalWrite(LIGHTS_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }    
  }
}

void TaskSensorValuesCode (void* parameters) {

  for (;;) {
    lighting_percentage = map(analogRead(ADC1), 0.0f, 4095.0f, 0, 100);
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
    Serial.print(light_sensor_reading);
    Serial.print(" (");
    Serial.print(100.00 - lighting_percentage);
    Serial.println("%)");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }  
}

void setup() {
  Serial.println("Please wait 2 seconds ...");
  delay(2000);
  Serial.begin(9600);
  Serial.println();
  
  Serial.println("Setting up GPIOs ...");

  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHTS_PIN, OUTPUT);
  digitalWrite(LIGHTS_PIN, LOW);

  Serial.println("GPIO setup done"); 

  xTaskCreate(TaskLightsAutoCode, "Lights auto", 4096, NULL, 4, &TaskLightsAuto); 

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
}

void loop() {
}