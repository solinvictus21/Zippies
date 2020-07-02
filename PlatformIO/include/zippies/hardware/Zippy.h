
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "ZippyWheel.h"

// #define LIMIT_ACCELERATION                                   1

class Zippy
{

private:
// #ifdef PLATFORM_TINYSCREEN
  // ZippyFace face;
// #endif
  bool inReverse = false;
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  MotorDriver motors;

#ifdef LIMIT_ACCELERATION
  double linearVelocity = 0.0d;
  double linearAcceleration = 0.0d;

  double angularVelocity = 0.0d;
  double angularAcceleration = 0.0d;

  void setTargetVelocities(double targetLinearVelocity, double targetAngularVelocity);
#endif

    void moveReverse(const ZMatrix2* relativeTarget);

public:
    Zippy();

    //for auto-tuning; only call this while the Zippy is stopped
    void setTunings(double p, double i, double d) {
        leftWheel.setTunings(p, i, d);
        rightWheel.setTunings(p, i, d);
    }

    void start();
    void setReverseMotion(bool r) { this->inReverse = r; }
    void move(const ZMatrix2* relativeTarget);
    void moveLinear(double relativeVelocity);
    void moveArc(double radius, double theta);
    void turn(double radius, double relativeOrientation);
    void turn(double relativeOrientation);

    void stop();

    // ZippyFace* getFace() { return &face; }

};

#endif
