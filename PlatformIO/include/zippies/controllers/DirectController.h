
#ifndef _DIRECTCONTROLLER_H_
#define _DIRECTCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/ZippyHardware.h"

class DirectController : public ZippyController
{

private:
  SensorFusor sensors;
  bool lighthouseReady = false;
  unsigned long previousPositionTimeStamp = 0;

  MotorDriver motors;

  int currentMoveIndex = 0;
  unsigned long currentMoveStartTime = 0;
  unsigned long currentMoveDeltaTime = 0;

  void startController(unsigned long currentTime);
  void startNextMove(unsigned long currentTime);
  void loopController(unsigned long currentTime);
  void stopController(unsigned long currentTime);

public:
  DirectController();
  void loop(unsigned long currentTime);

};

#endif
