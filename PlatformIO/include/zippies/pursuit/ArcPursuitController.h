
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
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    bool completeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    bool completeTurn(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);

public:
    ArcPursuitController()
    {}

    void continuePursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void stopPursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void stop();
    bool isStopped() { return currentMovementState == MovementState::Stopped; }


};

#endif