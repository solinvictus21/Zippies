

#ifndef _SCISSORPURSUITCONTROLLER_H_
#define _SCISSORPURSUITCONTROLLER_H_

#include "PursuitController.h"
#include "zippies/hardware/ZippyLA.h"

class ScissorPursuitController : public PursuitController
{

private:
    bool reverseDirection = false;
    SensorFusor* sensors;
    ZippyLA zippy;

    MovementState currentMovementState = MovementState::Stopped;
    unsigned long stateDowngradeCounter = 0;

    void executeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool clipVelocities);
    bool completeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    bool completeTurn( 
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);

    // bool completeMove(const ZMatrix2* relativeTargetPosition);
    // bool completeTurn(const ZMatrix2* relativeTargetPosition);

public:
    ScissorPursuitController(SensorFusor* s)
      : sensors(s)
    {}

    void setReverseDirection(bool r) { reverseDirection = r; }
    bool isReverseDirection() const { return reverseDirection; }
    void continuePursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void continueTurn(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    void stopPursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);
    // void stopPursuit(const ZMatrix2* relativeTargetPosition);
    void stop();
    bool isStopped() { return currentMovementState == MovementState::Stopped; }



};

#endif