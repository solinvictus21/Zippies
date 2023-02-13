
#include <Arduino.h>
#include <SPI.h>

#include "zippies/controllers/LighthouseController.h"
#include "zippies/controllers/DebugDisplayController.h"
#include "zippies/controllers/DrivingController.h"

LighthouseController::LighthouseController()
{
  // subController = new DebugDisplayController(&sensors);
  subController = new DrivingController(&sensors);
}

void LighthouseController::start()
{
  sensorsReady = false;
  previousPositionTimeStamp = 0;
  subControllerDeltaTime = 0;
  sensors.start();
}

void LighthouseController::loop(unsigned long deltaTime)
{
  subControllerDeltaTime += deltaTime;
  //wait until we have a stable signal
  if (!sensors.loop(deltaTime)) {
    if (sensorsReady) {
      // SerialUSB.println("Lighthouse signal lost.");
      sensorsReady = false;
      previousPositionTimeStamp = 0;
      subController->stop();
    }
    return;
  }
  else if (!sensorsReady) {
    // SerialUSB.println("Starting subcontroller.");
    //start the sub-controller, but don't loop it until the next update so that the first delta time is not zero
    sensorsReady = true;
    // subController->start(currentTime);
    subControllerDeltaTime = 0;
    subController->start();
    return;
  }

  //check if we have a position update
  unsigned long currentPositionTimeStamp = sensors.getPositionTimeStamp();
  if (currentPositionTimeStamp <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = currentPositionTimeStamp;

  // SerialUSB.print("lighthouse update: ");
  // SerialUSB.println(currentTime);

  subController->loop(subControllerDeltaTime);
  subControllerDeltaTime = 0;
}

void LighthouseController::stop()
{
  if (sensorsReady) {
    sensorsReady = false;
    previousPositionTimeStamp = 0;
    subController->stop();
  }
  sensors.stop();
}

LighthouseController::~LighthouseController()
{
  if (subController)
    delete subController;
}
