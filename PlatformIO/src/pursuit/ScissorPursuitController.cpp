
#include "zippies/pursuit/ScissorPursuitController.h"

// #define CLIP_VELOCITY_FACTOR_MAX      0.5
// #define CLIP_VELOCITY_DISTANCE_MAX    3.0

#define CLIP_VELOCITY_FACTOR_MAX      1.0
#define CLIP_VELOCITY_DISTANCE_MAX   10.0

void ScissorPursuitController::continuePursuit(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool reverseDirection)
{
    // SerialUSB.print("position: ");
    // sensors->getPosition()->printDebug();
    // SerialUSB.print("relative target: ");
    // relativeTargetPosition->printDebug();
    switch (currentMovementState) {
        case MovementState::Turning:
            //upgrade from turning to moving
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Moving;
            break;

        case MovementState::Stopped:
            //start moving
            zippy.start();
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Moving;
            break;
    }

    executeMove(relativeTargetPosition, relativeTargetVelocity, reverseDirection, true);
}

void ScissorPursuitController::stopPursuit(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool reverseDirection)
{
    //continue downgrading the movement state until stopped
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(relativeTargetPosition, relativeTargetVelocity, reverseDirection))
                return;

            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;

        case MovementState::Turning:
            if (!completeTurn(relativeTargetPosition, relativeTargetVelocity, reverseDirection))
                return;

            currentMovementState = MovementState::Stopped;
            zippy.stop();
    }
}

bool ScissorPursuitController::completeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool reverseDirection)
{
    if (relativeTargetPosition->getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        executeMove(relativeTargetPosition, relativeTargetVelocity, reverseDirection, false);
        return false;
    }

    return true;
}

bool ScissorPursuitController::completeTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool reverseDirection)
{
    // if (abs(relativeTargetVelocity->atan2()) > ANGULAR_EPSILON &&
        // stateDowngradeCounter)
    if (stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        zippy.setInput(sensors->getPositionDelta());
        zippy.turn(relativeTargetPosition->getY(), relativeTargetVelocity->atan2());
        return false;
    }

    return true;
}

void ScissorPursuitController::executeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool reverseDirection,
    bool clipVelocities)
{
    /*
    zippy.setInput(sensors->getPositionDelta());
    zippy.move(
        relativeTargetPosition->getY() >= 0.0 ? relativeTargetPosition->getD() : -relativeTargetPosition->getD(),
        2.0 * relativeTargetPosition->atan());
    */
    // /*
    /*
    let targetLinearDistance = (toPoint.getY() / 2.0) + (velocity.getY() / 4.0)
    */
    double linearTargetDistance = (relativeTargetPosition->getY() / 2.0) + (relativeTargetVelocity->getY() / 4.0);

    //first calculate the vector which represents the distance forward/backward to the midpoint of the velocity vector
    ZVector2 relativeEndPoint(
        relativeTargetPosition->getX() + (relativeTargetVelocity->getX() / 2.0),
        relativeTargetPosition->getY() + (relativeTargetVelocity->getY() / 2.0) - linearTargetDistance);
    if (reverseDirection) {
        relativeEndPoint.set(
            -linearTargetDistance * sin(relativeEndPoint.atan2()),
            linearTargetDistance - (linearTargetDistance * cos(relativeEndPoint.atan2())));
    }
    else {
        relativeEndPoint.set(
            linearTargetDistance * sin(relativeEndPoint.atan2()),
            linearTargetDistance + (linearTargetDistance * cos(relativeEndPoint.atan2())));
    }

    double linearVelocity, angularVelocity;
    if (relativeEndPoint.getX() == 0.0) {
        linearVelocity = relativeEndPoint.getY();
        angularVelocity = 0.0;
        if (clipVelocities) {
            linearVelocity -= constrain(linearVelocity * CLIP_VELOCITY_FACTOR_MAX,
                -CLIP_VELOCITY_DISTANCE_MAX, CLIP_VELOCITY_DISTANCE_MAX);
        }
    }
    else {
        angularVelocity = 2.0 * relativeEndPoint.atan();
        linearVelocity = angularVelocity * relativeEndPoint.getD2() / (2.0 * relativeEndPoint.getX());
        if (clipVelocities) {
            double clip = constrain(linearVelocity * CLIP_VELOCITY_FACTOR_MAX,
                -CLIP_VELOCITY_DISTANCE_MAX, CLIP_VELOCITY_DISTANCE_MAX);
            angularVelocity *= 1.0 - (clip / linearVelocity);
            linearVelocity -= clip;
        }
    }

    zippy.setInput(sensors->getPositionDelta());
    zippy.move(linearVelocity, angularVelocity);
    // */
}

void ScissorPursuitController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
