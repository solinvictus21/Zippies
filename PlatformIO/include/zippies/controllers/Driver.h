
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
    unsigned long remainingMovementTime = 0;
    bool reverseDirection = false;

    ZMatrix2 previousTargetPosition;
    ZMatrix2 targetPosition;

    //intermediate movement metrics
    ZVector2 shadowToTargetPosition;

    //output metrics
    double currentLinearVelocity = 0.0;
    double currentAngularVelocity = 0.0;
    ZMatrix2 shadowPosition;
    ZVector2 shadowVelocityVector;

    void calculateNewTarget();
    void holdAtTarget();

public:
    Driver()
    {}

    void reset();

    void start(const ZMatrix2* sp,
               double tx, double ty, double to,
               unsigned long movementTime);
    void start(double sx, double sy, double so,
               double tx, double ty, double to,
               unsigned long movementTime);

    void setTargetPosition(const ZMatrix2* tp,
                           unsigned long movementTime);
    void setTargetPosition(double targetX, double targetY, double targetO,
                           unsigned long movementTime);

    bool loop(unsigned long deltaTime);

    bool isHoldingPosition() const { return !remainingMovementTime; }
    
    const ZVector2* getShadowPosition() const { return &shadowPosition.position; }
    const ZVector2* getShadowVelocity() const { return &shadowVelocityVector; }
    bool isReverseDirection() const { return reverseDirection; }

};


#endif