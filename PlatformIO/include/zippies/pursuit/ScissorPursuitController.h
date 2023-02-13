

#ifndef _SCISSORPURSUITCONTROLLER_H_
#define _SCISSORPURSUITCONTROLLER_H_

#include "PursuitController.h"
#include "zippies/hardware/ZippyLA.h"
#include "zippies/hardware/Zippy.h"

class ScissorPursuitController : public PursuitController
{

private:
    // bool reverseDirection = false;
    SensorFusor* sensors;
    Zippy zippy;

    MovementState currentMovementState = MovementState::Stopped;
    unsigned long stateDowngradeCounter = 0;

    void executeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool reverseDirection,
        bool clipVelocities);
    bool completeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool reverseDirection);
    bool completeTurn( 
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool reverseDirection);

    /*
    void setReverseDirection(bool r) { reverseDirection = r; }
    bool isReverseDirection() const { return reverseDirection; }
    */

public:
    ScissorPursuitController(SensorFusor* s)
      : sensors(s)
    {}

    void continuePursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool reverseDirection);
    void stopPursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity,
        bool reverseDirection);
    // void stopPursuit(const ZMatrix2* relativeTargetPosition);
    void stop();
    bool isStopped() { return currentMovementState == MovementState::Stopped; }



};

#endif