
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "zippies/ZippyMath.h"

/**
 * The Driver class provides an outer navigation loop intended to mimic the way that a human driver
 * would attempt to arrive at a designated target pose (x/y/o). This implementation is based on
 * research and sample code described in the following document and related codebase.
 * 
 *     https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 *     https://github.com/h2ssh/Vulcan/blob/master/src/mpepc/control/kinematic_control_law.cpp
 * 
 */
class Driver
{

private:
    bool holdingPosition = false;
    bool reverseDirection = false;

    ZMatrix2 shadowPosition;
    ZVector2 shadowVelocityVector;
    ZMatrix2 previousTargetPosition;
    ZMatrix2 targetPosition;
    ZVector2 shadowToTargetPosition;

    double currentLinearVelocity = 0.0;
    double currentAngularVelocity = 0.0;

    void calculateNewTarget();

public:
    Driver()
    {}

    void reset();

    void start(const ZMatrix2* sp, double tx, double ty, double to);
    void start(double sx, double sy, double so, double tx, double ty, double to);

    void setTargetPosition(const ZMatrix2* tp);
    void setTargetPosition(double targetX, double targetY, double targetO);
    const ZMatrix2* getTargetPosition() const { return &targetPosition; }

    void update(unsigned long remainingTime);

    const ZVector2* getShadowPosition() const { return &shadowPosition.position; }
    const ZVector2* getShadowVelocity() const { return &shadowVelocityVector; }
    const ZVector2* getShadowToTargetPosition() const { return &shadowToTargetPosition; }
    double getDistanceToTarget() const { return shadowToTargetPosition.getD(); }
    bool isHoldingPosition() const { return holdingPosition; }
    bool isReverseDirection() const { return reverseDirection; }

    void holdPosition();

};


#endif