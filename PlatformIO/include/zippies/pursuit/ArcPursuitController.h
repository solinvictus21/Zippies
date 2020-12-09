
#ifndef _ARCPURSUITCONTROLLER_H_
#define _ARCPURSUITCONTROLLER_H_

#include "PursuitController.h"
#include "zippies/hardware/Zippy.h"

class ArcPursuitController : public PursuitController
{

private:
    Zippy zippy;

    MovementState currentMovementState = MovementState::Stopped;
    unsigned long stateDowngradeCounter = 0;

    void executeMove(
        const ZMatrix2* currentPosition,
        const ZVector2* targetPosition,
        const ZVector2* targetVelocity);

public:
    ArcPursuitController()
    {}

    void continuePursuit(
        const ZMatrix2* currentPosition,
        const ZVector2* targetPosition,
        const ZVector2* targetVelocity);
    void stopPursuit(
        const ZMatrix2* currentPosition,
        const ZVector2* targetPosition,
        const ZVector2* targetVelocity);
    void stop();
    bool isStopped() { return currentMovementState == MovementState::Stopped; }


};

#endif