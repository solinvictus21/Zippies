
#ifndef _LIGHTHOUSECONTROLLER_H_
#define _LIGHTHOUSECONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/SensorFusor.h"

class LighthouseController : public ZippyController
{

private:
  SensorFusor sensors;
  bool sensorsReady = false;
  unsigned long previousPositionTimeStamp = 0;
  unsigned long subControllerDeltaTime = 0;
  ZippyController* subController;

public:
  LighthouseController();

  void start();
  void loop(unsigned long deltaTime);
  void stop();

  ~LighthouseController();

};

#endif
