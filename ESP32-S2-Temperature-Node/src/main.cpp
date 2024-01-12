#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "secrets.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  
  Serial.begin(115200);
  Serial.println("Running setip ...");

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to Wi-Fi");  
  // Connect to wifi.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("#");
  }
  Serial.print("\nCONNECTED\nIP: ");  
  Serial.println(WiFi.localIP());

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  
  

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}