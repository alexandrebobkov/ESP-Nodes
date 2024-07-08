#include <Arduino.h>
//#include <Wire.h>

#include <WiFiClientSecure.h>

//#define BMP280
#define BME280

#ifdef BME280
#include <Adafruit_BME280.h>
#endif

#ifdef BMP280
#include <Adafruit_BMP280.h>
#endif

struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

void setup() {
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("Running setup ...");
  #ifdef BME280
  Serial.println("BME280");
  #endif
  #ifdef BMP280
  Serial.println("BMP280");
  #endif
  sensors_values.humidity = 0.0;
  sensors_values.pressure = 0.0;
  sensors_values.temperature = 0.0;

  #ifdef BME280
  Adafruit_BME280 bme;
  unsigned status = bme.begin(0x76);  // I2C slave address 0x76 (SDO set to GND)
  if (!status) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else {
    sensors_values.humidity = bme.readHumidity();
    sensors_values.pressure = bme.readPressure()  / 100.0F;
    sensors_values.temperature = bme.readTemperature();
    Serial.println("All values were measured.");
    Serial.print("Temperature: ");
    Serial.print(sensors_values.temperature);
    Serial.println();
    Serial.print("Pressure: ");
    Serial.print(sensors_values.pressure);
    Serial.println();
    Serial.print("Humidity: ");
    Serial.print(sensors_values.humidity);
    Serial.println();
  }
  #endif
  #ifdef BMP280
  Adafruit_BMP280 bmp;
  unsigned status = bmp.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else {
    sensors_values.pressure = bmp.readPressure()  / 100.0F;
    sensors_values.temperature = bmp.readTemperature();
    Serial.println("All values were measured.");
    Serial.print("Temperature: ");
    Serial.print(sensors_values.temperature);
    Serial.println();
    Serial.print("Pressure: ");
    Serial.print(sensors_values.pressure);
    Serial.println();
    Serial.print("Humidity: ");
    Serial.print(sensors_values.humidity);
    Serial.println();
  }
  #endif
}

void loop() {
  delay(500);
}