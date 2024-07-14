#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

#include "config.h"

Adafruit_BME280 bme;

// Define tasks.
TaskHandle_t TaskStatusLED, TaskSensorValues;

// Dummy task blinking built-in LED
void TaskStatusLEDCode (void * parameters) {
    Serial.print("Task 0 running on core # ");
    Serial.println(xPortGetCoreID());

    for (;;) {
      digitalWrite(SYS_LED_PIN, LOW);
      vTaskDelay(250);
      digitalWrite(SYS_LED_PIN, HIGH);
      vTaskDelay(250);
      digitalWrite(SYS_LED_PIN, LOW);
      vTaskDelay(250);
      digitalWrite(SYS_LED_PIN, HIGH);
      vTaskDelay(1500);                
    }
}

void setup() {
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
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(150);
      digitalWrite(LED_PIN, HIGH);
      delay(150);
      Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    }
  }
}

void loop() {
  
  digitalWrite(LED_PIN, LOW);
  delay(250);
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(250);
  digitalWrite(LED_PIN, HIGH);
  delay(750);
  Serial.println("Main loop");
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.println("°C");
  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println("%");
  Serial.print("Barometric Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" kPa");
  Serial.println("");
}