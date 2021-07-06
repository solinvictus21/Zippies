
#include <Arduino.h>
#include <SPI.h>

#include "zippies/controllers/LighthouseController.h"
#include "zippies/controllers/DebugDisplayController.h"
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/controllers/RoutineController.h"
#include "zippies/controllers/DrivingController.h"

LighthouseController::LighthouseController()
{
  // subController = new DebugDisplayController(&sensors);
  // subController = new MotorTuningController(&sensors);
  // subController = new PIDTuningController(&sensors);
  // subController = new RoutineController(&sensors);
  subController = new DrivingController(&sensors);
}

void LighthouseController::start(unsigned long currentTime)
{
  sensorsReady = false;
  previousPositionTimeStamp = 0;
  sensors.start();
}

void LighthouseController::loop(unsigned long currentTime)
{
  //wait until we have a stable signal
  if (!sensors.loop(currentTime)) {
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
    subController->start(currentTime);
    return;
  }

  //check if we have a position update
  unsigned long currentPositionTimeStamp = sensors.getPositionTimeStamp();
  if (currentPositionTimeStamp <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = currentPositionTimeStamp;

  // SerialUSB.print("lighthouse update: ");
  // SerialUSB.println(currentTime);

  subController->loop(currentTime);
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
