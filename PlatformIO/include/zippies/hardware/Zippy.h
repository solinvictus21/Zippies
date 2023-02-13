
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "ZippyWheel.h"
#include "zippies/math/ZPID.h"

class Zippy
{

private:
  bool inReverse = false;
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  MotorDriver motors;

public:
    Zippy();

    //for auto-tuning; only call this while the Zippy is stopped
    void setTunings(double p, double i, double d) {
        leftWheel.setTunings(p, i, d);
        rightWheel.setTunings(p, i, d);
    }

    void start();
    void setInput(const ZMatrix2* positionDelta);
    void setInput(double linearVelocity, double angularVelocity);
    void setReverseMotion(bool r) { this->inReverse = r; }
    void moveLinear(double relativeVelocity);
    void moveArc(double radius, double theta);
    void move(double linearVelocity, double angularVelocity);
    void turn(double yOffset, double angularVelocity);
    void stop();

};

#endif
