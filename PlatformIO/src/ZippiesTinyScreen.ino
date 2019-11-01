
#include <Wire.h>
#include "ZPrimaryController.h"

ZPrimaryController* controller;

void setup()
{
  Wire.begin();
  controller = new ZPrimaryController();
  controller->start(micros() / 1000);
}

void loop()
{
  controller->loop(micros() / 1000);
}
