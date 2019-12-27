
#ifndef _ZIPPYFACE_H_
#define _ZIPPYFACE_H_

#include <TinyScreen.h>

#define SCREEN_WIDTH_PIXELS      96
#define SCREEN_WIDTH_PIXELS_2    48
#define SCREEN_HEIGHT_PIXELS     64
#define SCREEN_HEIGHT_PIXELS_2   32

class ZippyFace
{

private:
  TinyScreen display;

  /*
  void drawBattery();
  int getBatteryLevel();
  void drawCoordinate(uint8_t x, uint8_t y, char* label, float value, int precision);
  void drawModeIndicator(uint8_t modeColor);
  */

public:
  ZippyFace();

  uint8_t getFontHeight() { return display.getFontHeight(); }

  void begin();
  void displayFace();
  void clearScreen();
  void displayText(uint8_t x, uint8_t y, const char* text);
  void displayLabelAndData(uint8_t x, uint8_t y, const char* label, double data);
  void displayLabelAndData(uint8_t x, uint8_t y, const char* label, double data, uint8_t digits);
  void displayLabelAndData(uint8_t x, uint8_t y, const char* label, int data);
  void end();

};

#endif
