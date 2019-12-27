
#ifndef _TARGETCONTROLLER_H_
#define _TARGETCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/Zippy.h"
#include "zippies/controllers/PIDTuningController.h"

class TargetController : public ZippyController
{

private:
  SensorFusor* sensors;
  Zippy zippy;
  ZippyController* subController;

public:
  TargetController(SensorFusor* sensors);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

  ~TargetController()
  {
    if (subController)
      delete subController;
  }

};

#endif
