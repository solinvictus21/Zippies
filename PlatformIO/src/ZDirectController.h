
#ifndef _ZDIRECTCONTROLLER_H_
#define _ZDIRECTCONTROLLER_H_

#include "ZController.h"
#include "MotorDriver.h"
#include "lighthouse/SensorFusor.h"

class ZDirectController : public ZController
{

private:
  SensorFusor* lighthouse;
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
  ZDirectController(SensorFusor* lighthouse);
  void loop(unsigned long currentTime);

};

#endif
