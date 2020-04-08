
#ifndef _FACEDISPLAY_H_
#define _FACEDISPLAY_H_

#include <TinyScreen.h>

class FaceDisplay
{

private:
  TinyScreen display;

  void displayDefaultFace();

public:
  FaceDisplay();

};

#endif
