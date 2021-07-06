#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

// #include <PID_v1.h>
#include "zippies/math/ZPID.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/PIDConfig.h"

class ZippyWheel
{

private:
    ZVector2 wheelOffset;
    double input = 0.0;
    ZPID wheelPID;

public:
    ZippyWheel(double wox, double woy)
      : wheelOffset(wox, woy),
        wheelPID(60, //0.0,
          PID_KP, PID_KI, PID_KD,
          -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
          false, false)
    {
    }

    void setTunings(double p, double i, double d) {
        wheelPID.setTunings(p, i, d);
    }

    void start() {
        wheelPID.start();
    }

    void setInput(double i) {
        input = i;
    }

    double moveStraight(double linearVelocity) {
        return wheelPID.compute(input, linearVelocity);
        // return wheelPID.compute(input, -linearVelocity);
    }

    double moveArc(double radius, double subtendedAngle) {
        return wheelPID.compute(input, (radius - wheelOffset.getX()) * subtendedAngle);
        // return wheelPID.compute(input, (wheelOffset.getX() - radius) * subtendedAngle);
    }

    void stop() {
        wheelPID.stop();
    }

};

#endif
