
#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include "zippies/math/ZPID.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/PIDConfig.h"

#define ZPID_CENTER_ON_ZERO

class ZippyWheel
{

private:
    double wheelOffset;
    double input = 0.0;
    ZPID wheelPID;

public:
    ZippyWheel(double wo)
      : wheelOffset(wo),
        wheelPID(60, //0.0,
          PID_KP, PID_KI, PID_KD,
          -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
          false, false)
    {
    }

    void setTunings(double p, double i, double d)
    {
        wheelPID.setTunings(p, i, d);
    }

    void start()
    {
        wheelPID.start();
    }

    void setInput(double i)
    {
        input = i;
    }

    double moveStraight(double linearVelocity)
    {
#ifdef ZPID_CENTER_ON_ZERO
        return wheelPID.compute(-linearVelocity, 0.0);
#else
        return wheelPID.compute(input, linearVelocity);
#endif
    }

    double moveArc(double radius, double subtendedAngle)
    {
#ifdef ZPID_CENTER_ON_ZERO
        return wheelPID.compute(-((radius - wheelOffset) * subtendedAngle), 0.0);
#else
        return wheelPID.compute(input, (radius - wheelOffset.getX()) * subtendedAngle);
#endif
    }

    void stop()
    {
        wheelPID.stop();
    }

};

#endif
