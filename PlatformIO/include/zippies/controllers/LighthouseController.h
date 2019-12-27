
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

  ZippyController* subController;

public:
  LighthouseController();

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);
  void stop();

  ~LighthouseController();

};

#endif
