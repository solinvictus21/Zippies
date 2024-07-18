
#include <Wire.h>
#include "Zippies.h"
#include "zippies/controllers/DebugDisplayController.h"

//starting a whole new scheme of using microseconds for timing
ZippyController* controller;
unsigned long startTimeMicros = 0;
unsigned long previousTimeMicros = 0;

void setup()
{
    Wire.begin();

    //retrieve the Zippy configuration
    // const ZippyConfig* zippyConfig = getZippyTestConfig();
    const ZippyConfig* zippyConfig = getZippyConfig();

    /* wait for the serial port monitor to connect
    while (!SerialUSB);
    SerialUSB.print("Zippy #: ");
    SerialUSB.println(zippyConfig->zippyNumber);
    // */
    
    // controller = new LighthouseController(zippyConfig);
    // controller = new MotorTuningController(zippyConfig);
    // previousTime = micros() / 1000;
    // controller->start();

    //starting a whole new scheme of using microseconds for timing
    controller = new RemoteDrivingController(zippyConfig);
    startTimeMicros = micros();
    previousTimeMicros = startTimeMicros;
    controller->start();
}

void loop()
{
    /* starting a whole new scheme of using microseconds for timing
    unsigned long currentTime = micros() / 1000;
    unsigned long deltaTime = currentTime - previousTime;
    previousTime = currentTime;

    // SerialUSB.print("Outer deltaTime: ");
    // SerialUSB.println(deltaTime);
    controller->loop(deltaTime);
    */

    unsigned long currentTimeMicros = micros() - startTimeMicros;
    unsigned long deltaTimeMicros = currentTimeMicros - previousTimeMicros;
    previousTimeMicros = currentTimeMicros;
    controller->loop(currentTimeMicros, deltaTimeMicros);
}
