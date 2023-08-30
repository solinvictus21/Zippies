
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#ifdef WEBOTS_SUPPORT
#include <webots/Robot.hpp>
using namespace webots;
#endif
#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "ZippyWheel.h"
#include "zippies/math/ZPID.h"

class Zippy
{

private:
    MotorDriver motors;
    ZippyWheel leftWheel;
    ZippyWheel rightWheel;

public:
#ifdef WEBOTS_SUPPORT
    Zippy(Robot* zippyWebots);
#else
    Zippy();
#endif

    //for auto-tuning; only call this while the Zippy is stopped
    void setTunings(double p, double i, double d) {
        leftWheel.setTunings(p, i, d);
        rightWheel.setTunings(p, i, d);
    }

    void start();
    void setInput(const ZMatrix2* positionDelta);
    void setInput(double linearVelocity, double angularVelocity);
    void moveLinear(double relativeVelocity);
    void move(double linearVelocity, double angularVelocity);
    void stop();

};

#endif
