#include "display.h"
#ifdef NO_DISPLAY
void displayInit() {}
void displayMessage(const char *message) {}
#else
// #include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "Display_ST7789.h"

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(14, 15, 21);

void displayInit() {
  LCD_Init();
  // pinMode(22, OUTPUT);
  // digitalWrite(22, HIGH);
  delay(100);
  tft.init(172, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.setCursor(0, 0);
  tft.setTextWrap(true);
  tft.print("Ready");
}
void displayMessage(const char *message) {
	// tft.startWrite();
  tft.fillScreen(ST77XX_BLUE);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(message);
	// tft.endWrite();
}
#endif
