
#pragma once
#include <TinyScreen.h>

class ZippyFace
{

private:
  TinyScreen display;

  void drawBattery();
  int getBatteryLevel();

  void drawCoordinate(uint8_t x, uint8_t y, char* label, float value, int precision);
  void drawModeIndicator(uint8_t modeColor);

public:
  ZippyFace();

  void start();
  void loop();
  void stop();
  
};


