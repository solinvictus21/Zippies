
#include <Wire.h>
#include "Zippies.h"

ZippyController* controller;
unsigned long previousTime = 0;

void setup()
{
    Wire.begin();

    // while (!SerialUSB);
    // SerialUSB.println("Started serial port.");
    initZippyConfiguration();

    previousTime = micros() / 1000;

    controller = new LighthouseController();
    controller->start();
}

void loop()
{
    unsigned long currentTime = micros() / 1000;
    unsigned long deltaTime = currentTime - previousTime;
    previousTime = currentTime;

    // SerialUSB.print("Outer deltaTime: ");
    // SerialUSB.println(deltaTime);
    controller->loop(deltaTime);
}
