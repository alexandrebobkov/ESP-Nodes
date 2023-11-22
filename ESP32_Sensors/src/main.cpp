#include <Arduino.h>
#include "config.h"

#ifdef BMP280
#include <Adafruit_BMP280.h>
#endif
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

// put function declarations here:
int myFunction(int, int);

struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  // BMP280
  #ifdef BMP280  
  unsigned status_bmp280;
  status_bmp280 = bmp.begin();
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

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}