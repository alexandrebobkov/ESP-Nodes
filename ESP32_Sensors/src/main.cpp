/*

ESP32 Node for reading seosors values and sending them over encrypted MQTT

By: Alexander Bobkov
Created:  July 28, 2023
Edited:   Nov 22, 2023

*/

#include <Arduino.h>
#include "config.h"                 // Set of configuration values

// Include library for specific sensor depending on configuration
// Environmental sensor BMP280: Temperature and pressure
#ifdef BMP280
#include <Adafruit_BMP280.h>
#endif
// Environmental sensor BME280: Temperature, pressure, and humidity
#ifdef BME280
#include <Adafruit_BME280.h>
#endif
// BME280
#ifdef BME280
// WaveShare BME280
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#endif
// BMP280
#ifdef BMP280
#define BMP_SCK   (18)
#define BMP_MISO  (19)
#define BMP_MOSI  (23)
#define BMP_CS    (5)
Adafruit_BMP280 bmp(BMP_CS);
#endif

// Struct for storing sensors values
struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

void setup()
{
  Serial.begin(115200);   // Define serial port speed

  // If BMP280 was defined in configuration file, complete setup procedure for BMP280 sensor
  #ifdef BMP280  
  unsigned status_bmp280;
  // Initialize sensor
  status_bmp280 = bmp.begin();
  // Repeat initialization attempt untill succeeded
  if (!status_bmp280) {
    Serial.println("Could not find BMP280");
    Serial.println(bmp.sensorID(),16);
    while (1);
  }
  else {
    Serial.println(bmp.sensorID(),16);
  }
  #endif

}

void loop() {

  // Read and display sensors values depending on configuration file
  #ifdef BMP280
  Serial.println("\n==== BMP-280 =============");
  sensors_values.temperature = (float)bmp.readTemperature();
  sensors_values.pressure = (float)bmp.readPressure();
  #endif

  // Display sensors values
  Serial.print("Temperature = ");
  Serial.println(sensors_values.temperature);
  Serial.print("Pressure = ");
  Serial.print(sensors_values.pressure / 100.0F);
  Serial.println(" Pa");
  delay(1000);

}