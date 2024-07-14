#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

#include "config.h"

Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Setting up GPIOs ...");

  pinMode(LED_PIN, OUTPUT);

  Serial.println("GPIO setup done");

  Serial.println("Setting up BME280 sensor");
  Wire.setPins(SDA_PIN, SCL_PIN);
  /*unsigned status = bme.begin(0x76); // 0x76
  if (!status) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(250);
      digitalWrite(LED_PIN, HIGH);
      delay(250);
    }
  }*/
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
}