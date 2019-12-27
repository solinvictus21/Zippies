
#include <Wire.h>
#include "Zippies.h"

ZippyController* controller;

void setup()
{
  Wire.begin();

  // while (!SerialUSB);
  // SerialUSB.println("Started serial port.");

  // controller = new DirectController();
  controller = new LighthouseController();

  controller->start(micros() / 1000);
}

void loop()
{
  controller->loop(micros() / 1000);
}
