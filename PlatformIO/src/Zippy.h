
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/KMatrix2.h"
#include "ZippyConfig.h"
#include "ZippyWheel.h"
#include "paths/ZPath.h"
#include "ZippyRoutine.h"

class Zippy
{

private:
#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  MotorDriver motors;
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;

  void processInput(const KMatrix2* positionDelta);
  void driveMotors();
  void stopPath();

public:
  Zippy();

  void start();
  void executeMove(const KMatrix2* positionDelta, KMatrix2* relativeTarget);
  void executeTurn(const KMatrix2* positionDelta, double relativeOrientation);
  void executeStop();
  void loop(unsigned long currentTime);

};

#endif
