
#ifndef _PATHFOLLOWINGCONTROLLER_H_
#define _PATHFOLLOWINGCONTROLLER_H_

#include "zippies/math/ZVector2.h"
#include "zippies/math/ZMatrix2.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/Zippy.h"
#include "zippies/hardware/ZippyWheel.h"
#include "zippies/ZippyRoutine.h"

class PathFollowingController
{

private:
    Zippy zippy;

    MovementState currentMovementState = MovementState::Stopped;
    unsigned long stateDowngradeCounter = 0;
    ZMatrix2 currentMovement;

    void executeMove();
    void executeTurn();
    void executeStop();

    void moveDirect();

public:
    PathFollowingController() {}

    Zippy* getZippy() { return &zippy; }
    void followPath(
        const ZMatrix2* currentPosition,
        const ZMatrix2* targetPosition,
        MovementState targetMovementState);
    void followPath(
        const ZMatrix2* currentPosition,
        const ZVector2* targetPosition,
        const ZVector2* targetVelocity);
    void stopPath(
        const ZMatrix2* currentPosition,
        const ZVector2* targetPosition,
        const ZVector2* targetVelocity);

    void stop();
    bool isStopped() { return currentMovementState == MovementState::Stopped; }

};

#endif
