
#include "zippies/basestation/BaseStationController.h"

BaseStationController baseStationController;

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("Started serial communication.");

  baseStationController.start();
}

void loop()
{
  baseStationController.loop();
}
