
#ifndef _ZDIRECTCONTROLLER_H_
#define _ZDIRECTCONTROLLER_H_

#include "ZController.h"
#include "MotorDriver.h"
#include "lighthouse/Lighthouse.h"

class ZDirectController : public ZController
{

private:
  Lighthouse* lighthouse;
  bool lighthouseReady = false;

  MotorDriver motors;

  int currentMoveIndex = 0;
  unsigned long currentMoveStartTime = 0;
  unsigned long currentMoveDeltaTime = 0;

  void startController(unsigned long currentTime);
  void startNextMove(unsigned long currentTime);
  void loopController(unsigned long currentTime);
  void stopController(unsigned long currentTime);

public:
  ZDirectController(Lighthouse* lighthouse);
  void loop(unsigned long currentTime);

};

#endif
