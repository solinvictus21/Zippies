
#include <Wire.h>
#include "Zippies.h"

ZippyController* controller;
unsigned long previousTime = 0;

void setup()
{
    Wire.begin();

    //retrieve the Zippy configuration
    const ZippyConfig* zippyConfig = getZippyTestConfig();
    // const ZippyConfig* zippyConfig = getZippyConfig();

    /* wait for the serial port monitor to connect
    while (!SerialUSB);
    SerialUSB.print("Zippy #: ");
    SerialUSB.println(zippyConfig->zippyNumber);
    // */
    
    controller = new LighthouseController(zippyConfig);
    // controller = new MotorTuningController(zippyConfig);
    previousTime = micros() / 1000;
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
