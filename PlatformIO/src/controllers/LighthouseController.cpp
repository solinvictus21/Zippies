
#include "zippies/controllers/LighthouseController.h"
#include "zippies/controllers/DrivingController.h"
#include "zippies/controllers/MotorTuningController.h"

#ifdef WEBOTS_SUPPORT
LighthouseController::LighthouseController(Supervisor* zippyWebots)
  : sensors(zippyWebots)
{
  subController = new DrivingController(zippyWebots, &sensors);
}
#else
LighthouseController::LighthouseController()
  : sensors()
{
  subController = new DrivingController(&sensors);
  // subController = new MotorTuningController(&sensors);
}
#endif

void LighthouseController::start()
{
  sensorsReady = false;
  previousPositionTimeStamp = 0;
  subControllerDeltaTime = 0;
  sensors.start();
}

bool LighthouseController::loop(unsigned long deltaTime)
{
  //wait until we have a stable signal
  if (!sensors.loop(deltaTime)) {
    if (sensorsReady) {
      //lost the lighthouse signal
      sensorsReady = false;
      previousPositionTimeStamp = 0;
      subController->stop();
    }
    return false;
  }
  else if (!sensorsReady) {
    //acquired the lighthouse signal
    //start the sub-controller, but don't loop it until the next update so that the first delta time is not zero
    sensorsReady = true;
    // subController->start(currentTime);
    subControllerDeltaTime = 0;
    subController->start();
    return false;
  }

  //check if we have a position update
  subControllerDeltaTime += deltaTime;
  unsigned long currentPositionTimeStamp = sensors.getPositionTimeStamp();
  if (currentPositionTimeStamp <= previousPositionTimeStamp)
      return false;
  previousPositionTimeStamp = currentPositionTimeStamp;

  subController->loop(subControllerDeltaTime);
  subControllerDeltaTime = 0;
  return true;
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
