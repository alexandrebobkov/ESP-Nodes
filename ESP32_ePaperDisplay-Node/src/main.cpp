#include <Arduino.h>
//#include <Fonts/FreeMonoBold9pt7b.h>
#include "ePaper.h"

// put function declarations here:
int myFunction(int, int);

void setup() {

  pinMode(12, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Running setup ...");

  // Initialize ePaper display
  display.init(115200);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMono9pt7b);
  display.setCursor(205, 10);
  display.print("ESP32_DisplayNode");

  display.setCursor(5, 5);
  display.setFont(&TomThumb);
  display.print("10.100.50.105");
  // Add UI elements
  // Draw axis: (x1,y1) @ 5, 100 and width-2*margin_x height-2*margin_y 140
  display.drawRect(5, 155, display.width()-10, display.height()-160, GxEPD_RED);
  display.setFont(&TomThumb);
  for (int i = 0; i < display.width()-10; i+=50) {
    display.setCursor(i, display.height());
    display.print(i);
  }

  // Calibration lines
  /*display.drawLine(0, 100, display.width(), 100, GxEPD_BLACK);
  display.drawLine(0, 200, display.width(), 200, GxEPD_BLACK);
  display.drawLine(200, 0, 200, display.height(), GxEPD_BLACK);*/

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