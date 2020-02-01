
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "ZippyWheel.h"

class Zippy
{

private:
#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  MotorDriver motors;

  void moveLinear(double relativeVelocity);

public:
  Zippy();

  //for auto-tuning; only call this while the Zippy is stopped
  void setTunings(double p, double i, double d) {
    leftWheel.setTunings(p, i, d);
    rightWheel.setTunings(p, i, d);
  }

  void start();
  void move(const KMatrix2* relativeTarget);
  void turn(double relativeOrientation);
  void stop();

  ZippyFace* getFace() { return &face; }

};

#endif
