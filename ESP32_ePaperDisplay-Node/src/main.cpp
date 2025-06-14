#include <Arduino.h>
#include "ePaper.h"

// put function declarations here:
void loop();
void setup();

void setup() {

  pinMode(8, OUTPUT); // 12
  
  Serial.begin(115200);
  Serial.println("Running setup ...");

  // Initialize ePaper display
  display.init(115200);
  display.fillScreen(GxEPD_BLACK);
  display.update();
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMono9pt7b);
  //display.setCursor(205, 10);
  //display.print("ESP32_DisplayNode"); // Takes 195 pixels in width

  display.setCursor(5, 5);
  display.setFont(&TomThumb);
  display.print("10.100.50.105");
  display.update();
  // Add UI elements
  // Draw axis: (x1,y1) @ 5, 100 and width-2*margin_x height-2*margin_y
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
  digitalWrite(8, HIGH); //12
  delay(750);
  Serial.println("LED OFF");
  digitalWrite(8, LOW); //12
  delay(750);
}