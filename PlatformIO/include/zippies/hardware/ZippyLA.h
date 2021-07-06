
#ifndef _ZIPPYLA_H_
#define _ZIPPYLA_H_

#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "ZippyWheel.h"
#include "zippies/math/ZPID.h"

class ZippyLA
{

private:
    bool inReverse = false;
    double linearInput = 0.0;
    double angularInput = 0.0;
    ZPID linearPID;
    ZPID angularPID;
    MotorDriver motors;

public:
    ZippyLA();

    void start();
    void setReverseMotion(bool r) { this->inReverse = r; }
    void setInput(const ZMatrix2* positionDelta);
    void move(double linearVelocity, double angularVelocity);
    void turn(double yOffset, double angularVelocity);
    // void turn(double angularVelocity);
    void stop();

};

#endif
