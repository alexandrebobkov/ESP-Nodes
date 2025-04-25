/*

  By:           Alexander Bobkov
  Date:         April 20, 2025
  Description:  This is an example of how to use the GxEPD library to display text and graphics on an ePaper display powered by the ESP32-C3 microcontroller.

  BUSY -> GPIO3
  RST  -> GPIO2
  DC   -> GPIO1
  CS   -> GPIO7 (SS)
  CLK  -> GPIO4 (SCK)
  DIN  -> GPIO6 (MOSI)

  */
#include <Arduino.h>
#include "ePaper.h"

// put function declarations here:
void loop();
void setup();

int16_t tbx, tby; 
uint16_t tbw, tbh;

void setup() {

    // Blink on-board LED once upon initialization
  pinMode(8, OUTPUT);
  Serial.println("LED ON");
  digitalWrite(8, HIGH);
  delay(1000);
  Serial.println("LED OFF");
  digitalWrite(8, LOW);
  delay(1000);
  
  Serial.begin(115200);
  Serial.println("Running setup ...");

  // Initialize ePaper display
  display.init(115200);
  display.fillScreen(GxEPD_BLACK);
  display.update();
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  //display.setTextColor(GxEPD_WHITE);
  display.setFont(&FreeMono9pt7b);
  //display.setCursor(205, 10);
  //display.print("ESP32_DisplayNode"); // Takes 195 pixels in width

  display.setCursor(5, 5);
  display.setFont(&TomThumb);
  display.print("10.100.50.105");
  display.update();

  // Display fonts
  display.setCursor(10, 10);
  display.setFont(&Picopixel);
  display.print("Picopixel");
  //display.update();

  display.drawBitmap(24, 24, gridicons_computer, 24, 24, GxEPD_BLACK);

  display.setCursor(10, 150);
  display.setFont(&FreeSans9pt7b);
  display.print("Files Storage");
  display.setCursor(10, 160);
  display.setFont(&FreeSerif9pt7b);
  display.print("Archives Storage");

// Display text nicely centered on the display
  display.setFont(&FreeSerif12pt7b);                                          // Specify the font family & size
  display.getTextBounds("SERVER DASHBOARD", 10, 100, &tbx, &tby, &tbw, &tbh);   // Request width of text to be displayed
  display.setCursor((display.width() - tbw) / 2, 100);                        // Calculate x-coordinate value
  display.print("SERVER DASHBOARD");                                            // Print the text
  //display.update();

  //display.drawBitmap(200, 200, gridicons_bug, 24, 24, GxEPD_BLACK);
  //display.invertDisplay(true);
  //display.drawBitmap(150, 150, gridicons_cloud, 24, 24, GxEPD_WHITE);
  
  // Add UI elements
  // Draw axis: (x1,y1) @ 5, 100 and width-2*margin_x height-2*margin_y
  display.drawRect(5, 155, display.width()-10, display.height()-160, GxEPD_RED);
  display.drawRect(6, 156, display.width()-12, display.height()-162, GxEPD_RED);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&TomThumb);
  for (int i = 0; i < display.width()-10; i+=50) {
    display.setCursor(i, display.height());
    display.print(i);
  }

  // Calibration lines
  /*display.drawLine(0, 100, display.width(), 100, GxEPD_BLACK);
  display.drawLine(0, 200, display.width(), 200, GxEPD_BLACK);
  display.drawLine(200, 0, 200, display.height(), GxEPD_BLACK);*/

  //display.invertDisplay(true);
  display.update();
}

void loop() {
  Serial.println("LED ON");
  digitalWrite(8, HIGH);
  delay(250);
  Serial.println("LED OFF");
  digitalWrite(8, LOW);
  delay(750);
}