
#ifndef _PATHFOLLOWINGCONTROLLER_H_
#define _PATHFOLLOWINGCONTROLLER_H_

#include "zippies/math/KMatrix2.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/Zippy.h"
#include "zippies/hardware/ZippyWheel.h"
#include "zippies/ZippyRoutine.h"

class PathFollowingController
{

private:
  Zippy zippy;

  MovementState currentMovementState = MovementState::Stopped;
  unsigned long stateDowngradeIterationCount = 0;
  KMatrix2 currentMovement;

  void move();
  void moveDirect();

public:
  PathFollowingController() {}

  Zippy* getZippy() { return &zippy; }
  void followPath(
      const KMatrix2* currentPosition,
      const KMatrix2* targetPosition,
      MovementState targetMovementState);
  void stop();

  bool isStopped() { return currentMovementState == MovementState::Stopped; }

};

#endif
