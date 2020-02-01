
#include "zippies/controllers/LighthouseController.h"
#include "zippies/controllers/DebugDisplayController.h"
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/controllers/PIDTuningController.h"

LighthouseController::LighthouseController()
{
  // subController = new DebugDisplayController(&sensors);
  // subController = new MotorTuningController(&sensors);
  subController = new PIDTuningController(&sensors);
}

void LighthouseController::start(unsigned long currentTime)
{
  SerialUSB.begin(115200);
  // while (!SerialUSB);
  SerialUSB.println("Serial port started.");

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
    sensorsReady = true;
    subController->start(currentTime);
  }

  //check if we have a position update
  unsigned long currentPositionTimeStamp = sensors.getPositionTimeStamp();
  if (currentPositionTimeStamp <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = currentPositionTimeStamp;

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
