
#include "zippies/controllers/TargetController.h"

TargetController::TargetController(SensorFusor* s)
  : sensors(s)
{
  subController = new PIDTuningController(sensors, &zippy);
}

void TargetController::start(unsigned long startTime)
{
  zippy.setTargetPosition(sensors->getPosition());
  zippy.start();
  subController->start(startTime);
}

void TargetController::loop(unsigned long currentTime)
{
  zippy.setCurrentPosition(sensors->getPosition());
  subController->loop(currentTime);
  zippy.loop();
}

void TargetController::stop()
{
  subController->stop();
  zippy.stop();
}
