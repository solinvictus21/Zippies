
#include <Wire.h>
#include "Zippies.h"

ZippyController* controller;

void setup()
{
    Wire.begin();

    // while (!SerialUSB);
    // SerialUSB.println("Started serial port.");
    initZippyConfiguration();

    // controller = new DirectController();
    controller = new LighthouseController();
    // controller = new BaseStationController();

    controller->start(micros() / 1000);
}

void loop()
{
    controller->loop(micros() / 1000);
}
