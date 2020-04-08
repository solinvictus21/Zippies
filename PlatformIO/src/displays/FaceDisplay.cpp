
#include "zippies/displays/FaceDisplay.h"
#include "zippies/displays/FaceGraphics.h"

FaceDisplay::FaceDisplay()
  : display(TinyScreenPlus)
{
  display.begin();
  display.setBrightness(4);
  display.setFlip(true);

  displayDefaultFace();
}

void FaceDisplay::displayDefaultFace()
{
  //displaying the face is a bit processor-intensive, so we just do it once at the start, since it doesn't currently change
  display.startData();
  display.writeBuffer((uint8_t*)FACE_HAPPY, 6144);
  display.endTransfer();
}
