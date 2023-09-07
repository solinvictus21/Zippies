
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#ifdef WEBOTS_SUPPORT
#include <webots/Node.hpp>
using namespace webots;
#endif
#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/ZippyRoutines.h"
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
    Zippy(const ZippyConfig* zippyConfig, Robot* zippyWebots);
#else
    Zippy(const ZippyConfig* zippyConfig);
#endif

    //for auto-tuning; only call this while the Zippy is stopped
    void setTunings(double p, double i, double d) {
        leftWheel.setTunings(p, i, d);
        rightWheel.setTunings(p, i, d);
    }

    void start();
    void setInput(const ZMatrix2* positionDelta);
    void move(double linearVelocity, double angularVelocity);
    void stop();

};

#endif
