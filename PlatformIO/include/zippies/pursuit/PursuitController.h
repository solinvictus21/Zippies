
#ifndef _PURSUITCONTROLLER_H_
#define _PURSUITCONTROLLER_H_

#include "zippies/ZippyMath.h"
#include "zippies/hardware/Zippy.h"

class PursuitController
{

protected:
    double linearVelocity = 0.0;
    double angularVelocity = 0.0;

public:
    PursuitController() {}

    virtual void executeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity) = 0;

    double getLinearVelocity() const { return linearVelocity; }
    double getAngularVelocity() const { return angularVelocity; }

    virtual ~PursuitController() {};

};

#endif