#include <Arduino.h>
//#include <Fonts/FreeMonoBold9pt7b.h>
#include "ePaper.h"

// put function declarations here:
int myFunction(int, int);

void setup() {

  pinMode(12, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Running setup ...");

  display.init(115200);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(10, 20);
  display.print("ESP32 Display Node");
  display.update();
}

void loop() {
  Serial.println("LED ON");
  digitalWrite(12, HIGH);
  delay(750);
  Serial.println("LED OFF");
  digitalWrite(12, LOW);
  delay(750);
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}