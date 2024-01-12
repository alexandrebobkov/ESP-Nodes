#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "secrets.h"

// Define tasks
TaskHandle_t Status_Task; // Status LED task
unsigned int t = 125;     // Blink delay time

// Declare functions
int myFunction(int, int);
void StatusCode (void* parameters);

void setup() {
  
  // Initialize setup
  Serial.begin(115200);
  Serial.println("Running setip ...");
  // Set status LED blink rapidly during setup phase
  t = 125;
  // Start LED status task
  xTaskCreatePinnedToCore(StatusCode, "Status LED", 1000, NULL, 2, &Status_Task, 0);

  // Initialize Wi-Fi connection
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to Wi-Fi");  
  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("#");
  }
  // If Wi-Fi connection was successfull, then set status LED blink slowly
  delay(1500);
  t = 1500;
  Serial.print("\nCONNECTED\nIP: ");  
  Serial.println(WiFi.localIP());

  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
}

// Status LED task code
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