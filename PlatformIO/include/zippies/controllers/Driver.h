
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "zippies/ZippyMath.h"

class Driver
{

private:
    bool reverseDirection = false;

    ZMatrix2 shadowPosition;
    ZVector2 shadowVelocityVector;
    ZMatrix2 targetPosition;
    ZVector2 shadowToTargetPosition;

    double currentLinearVelocity = 0.0;
    double currentAngularVelocity = 0.0;

public:
    Driver()
    {}

    void reset();

    // void setAnchorPosition(double anchorX, double anchorY, double anchorO) { anchorPosition.set(anchorX, anchorY, anchorO); }
    // void setAnchorPosition(const ZMatrix2* anchor) { anchorPosition.set(anchor); }
    // const ZMatrix2* getAnchorPosition() const { return &anchorPosition; }

    void setReverseDirection(bool rv) { reverseDirection = rv; }
    bool isReverseDirection() const { return reverseDirection; }
    
    void setShadowPosition(const ZMatrix2* sp);
    void setShadowPosition(double sx, double sy, double so);
    const ZMatrix2* getShadowPosition() const { return &shadowPosition; }

    const ZMatrix2* getTargetPosition() const { return &targetPosition; }
    void setTargetPosition(const ZMatrix2* tp);
    void setTargetPosition(double targetX, double targetY, double targetO);

    void update(unsigned long remainingTime);

    const double getLinearVelocity() const { return currentLinearVelocity; }
    const double getAngularVelocity() const { return currentAngularVelocity; }
    const ZVector2* getShadowVelocity() const { return &shadowVelocityVector; }
    double getDistanceToTarget() const { return shadowToTargetPosition.getD(); }

    void stop();


};


#endif