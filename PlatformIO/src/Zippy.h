
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyController.h"
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/Lighthouse.h"
#include "lighthouse/KPosition.h"
#include "ZippyConfig.h"
#include "ZippyWheel.h"
#include "commands/PathMove.h"

class Zippy : public ZippyController
{

private:
  KPosition targetPosition;
  PathMove* movementPath;

#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  Lighthouse lighthouse;
  bool lighthouseReady = false;

  MotorDriver motors;

  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  bool isMoving = false;

  unsigned long lastUpdateTime;

  void processInput();

  void moveArc(const KPosition* relativeTargetPosition);
  void moveBiArc(KPosition* relativeTargetPosition);
  void reversePlotBiArc(KPosition* relativeTargetPosition);
  void plotBiArc(KPosition* relativeTargetPosition);
  double centerTurnRadius(double distanceDelta, double orientationDelta);

  double saturate(double a, double b);
  void driveMotors();

public:
  Zippy(
    double startingX,
    double startingY,
    double startingOrientation,
    PathMove* movementPath);

  //start ZippyController interface
  const KPosition* getTargetPosition() const { return &targetPosition; }
  void startMoving();
  void move(double x, double y, double orientation);
  void turn(double orientation);
  void stopMoving();
  void waitForPreamble() { lighthouse.clearPreambleFlag(); }
  bool foundPreamble() const { return lighthouse.foundPreamble(); }
  //end ZippyController interface

  const Lighthouse* getLighthouse() const { return &lighthouse; }

#ifdef PLATFORM_TINYSCREEN
  const ZippyFace* getFace() { return &face; }
#endif

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);

  const ZippyWheel* getLeftWheel() const { return &leftWheel; }
  const ZippyWheel* getRightWheel() const { return &rightWheel; }

  ~Zippy()
  {
    delete movementPath;
  }

};

#endif
