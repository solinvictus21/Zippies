
#ifndef _ZIPPYFACE_H_
#define _ZIPPYFACE_H_
#include <TinyScreen.h>

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

  void start();

};

#endif
