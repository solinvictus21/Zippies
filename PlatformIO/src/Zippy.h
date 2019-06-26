
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/Lighthouse.h"
#include "lighthouse/KPosition.h"
#include "ZippyConfig.h"
#include "ZippyWheel.h"
#include "commands/PathMove.h"

#include "paths/ZPathPlanner.h"

typedef enum _ZippyState
{
  WaitingForLighthouse,
  MovingToInitialPosition,
  SyncingWithPreamble,
  Executing
} ZippyState;

class Zippy
{

private:
  KPosition startPosition;

  ZippyState currentState = WaitingForLighthouse;
  int routineIndex = 0;
  ZPath* currentPath = NULL;
  unsigned long currentPathStartTime = 0;
  unsigned long currentPathDeltaTime = 0;
  KPosition targetPosition;

#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  Lighthouse lighthouse;
  // bool lighthouseReady = false;

  MotorDriver motors;

  ZippyWheel leftWheel;
  ZippyWheel rightWheel;

  unsigned long lastUpdateTime;

  void processCurrentPath(unsigned long currentTime);
  void planNextPath(unsigned long currentTime);
  void processInput();
  void executeMove();
  void driveMotors();
  double saturate(double a, double b);

public:
  Zippy(
    double startingX,
    double startingY,
    double startingOrientation);

  const KPosition* getTargetPosition() const { return &targetPosition; }

  const Lighthouse* getLighthouse() const { return &lighthouse; }

#ifdef PLATFORM_TINYSCREEN
  const ZippyFace* getFace() { return &face; }
#endif

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);

  const ZippyWheel* getLeftWheel() const { return &leftWheel; }
  const ZippyWheel* getRightWheel() const { return &rightWheel; }

  /*
  ~Zippy()
  {
    delete movementPath;
  }
  // */

};

#endif
