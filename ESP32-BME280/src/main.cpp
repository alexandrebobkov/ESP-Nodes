#include <Arduino.h>

#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <WiFiClientSecure.h>

//#define BME280

struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

void setup() {
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("Running setup ...");
  sensors_values.humidity = 0.0;
  sensors_values.pressure = 0.0;
  sensors_values.temperature = 0.0;

  byte error, address;
  int dev = 0;
  for (address = 0x1; address < 0x127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      dev++;
    }
    else if (error == 4) {
      Serial.println("Unknown error at address 0x");
      Serial.println(address, HEX);
    }
    delay(500);
  }

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
  }
  #endif
  #ifdef BMP280
  Adafruit_BMP280 bmp;
  unsigned status = bmp.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else {}
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
}