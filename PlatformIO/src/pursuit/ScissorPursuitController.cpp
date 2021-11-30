
#include "zippies/pursuit/ScissorPursuitController.h"

#define CLIP_VELOCITY_FACTOR_MAX      0.5
#define CLIP_VELOCITY_DISTANCE_MAX    3.0

void ScissorPursuitController::continuePursuit(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
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

    executeMove(relativeTargetPosition, relativeTargetVelocity, true);
}

void ScissorPursuitController::continueTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    //immediately downgrade the movement state to turning
    switch (currentMovementState) {
        case MovementState::Moving:
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;
            break;

        case MovementState::Stopped:
            //start moving
            zippy.start();
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;
            break;
    }

    zippy.setInput(sensors->getPositionDelta());
    zippy.turn(relativeTargetPosition->getY(), relativeTargetVelocity->atan2());
}

void ScissorPursuitController::stopPursuit(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    //continue downgrading the movement state until stopped
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(relativeTargetPosition, relativeTargetVelocity))
                return;

            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;

        case MovementState::Turning:
            if (!completeTurn(relativeTargetPosition, relativeTargetVelocity))
                return;

            currentMovementState = MovementState::Stopped;
            zippy.stop();
    }
}

bool ScissorPursuitController::completeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    if (relativeTargetPosition->getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        executeMove(relativeTargetPosition, relativeTargetVelocity, false);
        return false;
    }

    return true;
}

bool ScissorPursuitController::completeTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
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
    bool clipVelocities)
{
    double linearTargetDistance = (relativeTargetPosition->getY() / 2.0) + (relativeTargetVelocity->getY() / 4.0);

    //first calculate the vector which represents the distance forward/backward to the midpoint of the velocity vector
    ZVector2 relativeEndPoint = ZVector2(
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
}

void ScissorPursuitController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
