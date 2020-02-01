
#ifndef _PATHFOLLOWINGCONTROLLER_H_
#define _PATHFOLLOWINGCONTROLLER_H_

#include "zippies/math/KMatrix2.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/Zippy.h"
#include "zippies/hardware/ZippyWheel.h"
#include "zippies/ZippyPaths.h"

class PathFollowingController
{

private:
  Zippy zippy;

  MovementState currentMovementState = MovementState::Stopped;
  KMatrix2 currentMovement;

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
