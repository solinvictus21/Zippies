
#ifndef _PUREPURSUITCONTROLLER_H_
#define _PUREPURSUITCONTROLLER_H_

#include "PursuitController.h"
#include "zippies/math/ZVector2.h"
#include "zippies/math/ZMatrix2.h"
#include "zippies/hardware/Zippy.h"
#include "zippies/ZippyRoutine.h"

class PurePursuitController : public PursuitController
{

private:
    Zippy zippy;

    MovementState currentMovementState = MovementState::Stopped;
    unsigned long stateDowngradeCounter = 0;
    // ZMatrix2 currentMovement;
    // ZVector2 currentVelocityTarget;

    void clipMove();

    void continueMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    bool completeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void forward(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool completingMove);
    void continueTurn(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    bool completeTurn(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void completeStop(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);

public:
    PurePursuitController() {}

    Zippy* getZippy() { return &zippy; }

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
