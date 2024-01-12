#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "secrets.h"

// Define tasks
TaskHandle_t Status_Task;
unsigned int t = 125;

// put function declarations here:
int myFunction(int, int);
void StatusCode (void* parameters);

void setup() {
  
  Serial.begin(115200);
  Serial.println("Running setip ...");
  t = 125;
  xTaskCreatePinnedToCore(StatusCode, "Status LED", 1000, NULL, 2, &Status_Task, 0);

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to Wi-Fi");  
  // Connect to wifi.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("#");
  }
  delay(1500);
  t = 1500;
  Serial.print("\nCONNECTED\nIP: ");  
  Serial.println(WiFi.localIP());

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
}

void StatusCode (void* parameters)
{
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(t);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(t);
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}