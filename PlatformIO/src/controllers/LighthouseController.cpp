
#include "zippies/controllers/LighthouseController.h"
#include "zippies/controllers/DebugDisplayController.h"
#include "zippies/controllers/TargetController.h"
#include "zippies/controllers/MotorTuningController.h"

LighthouseController::LighthouseController()
{
  // subController = new DebugDisplayController(&sensors);
  subController = new TargetController(&sensors);
  // subController = new MotorTuningController(&sensors);
  // subController = new PIDTuningController(&sensors);
}

void LighthouseController::start(unsigned long currentTime)
{
  SerialUSB.begin(115200);

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

  //check if we have a position update
  if (sensors.getPositionTimeStamp() <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = sensors.getPositionTimeStamp();

  if (!sensorsReady) {
    // SerialUSB.println("Lighthouse signal locked.");
    sensorsReady = true;
    subController->start(currentTime);
    return;
  }

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
