
#include "zippies/pursuit/PurePursuitController.h"
#include <Arduino.h>

#define MIN_DELTA_TARGET_EPSILON                      0.1
#define MAX_DELTA_TARGET_EPSILON                      0.5

#define DEFAULT_LOOK_AHEAD_DISTANCE                  20.0

double pad(double value, double epsilon)
{
    if (value == 0.0)
        return 0.0;

    return value < 0.0
        ? min(value, -epsilon)
        : max(value, epsilon);
}

/*
void PurePursuitController::followPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    currentMovement.set(
        targetPosition->getX(), targetPosition->getY(),
        targetVelocity->atan2());
    currentMovement.unconcat(currentPosition);
    currentVelocityTarget.set(
        targetPosition->getX() + (targetVelocity->getX() / 60.0),
        targetPosition->getY() + (targetVelocity->getY() / 60.0));
    currentVelocityTarget.subtractVector(&currentPosition->position);
    currentVelocityTarget.rotate(currentPosition->orientation.get());

    continueMove(&currentMovement);
}

void PurePursuitController::stopPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    currentMovement.set(
        targetPosition->getX(), targetPosition->getY(),
        targetVelocity->atan2());
    currentMovement.unconcat(currentPosition);
    completeStop(&currentMovement);
}
*/

void PurePursuitController::continuePursuit(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
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

    ZVector2 relativeTargetPosition(targetPosition);
    relativeTargetPosition.subtractVector(&currentPosition->position);
    relativeTargetPosition.unrotate(&currentPosition->orientation);
    relativeTargetPosition.multiply(0.5);
    // ZVector2 relativeTargetVelocity(targetVelocity);
    // relativeTargetVelocity.unrotate(&currentPosition->orientation);

    if (relativeTargetPosition.getX() == 0.0) {
      zippy.moveLinear(relativeTargetPosition.getY());
      return;
    }

    double movementRadius = relativeTargetPosition.getD2() / (2.0 * relativeTargetPosition.getX());
    double movementTheta = 2.0 * relativeTargetPosition.atan();
    zippy.moveArc(movementRadius, movementTheta);
}

void PurePursuitController::stopPursuit(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    ZVector2 relativeTargetPosition(targetPosition);
    relativeTargetPosition.subtractVector(&currentPosition->position);
    relativeTargetPosition.unrotate(&currentPosition->orientation);
    ZVector2 relativeTargetVelocity(targetVelocity);
    relativeTargetVelocity.unrotate(&currentPosition->orientation);

    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(&relativeTargetPosition, &relativeTargetVelocity))
                return;

        case MovementState::Turning:
            if (!completeTurn(&relativeTargetPosition, &relativeTargetVelocity))
                return;
    }

    stop();
}

void PurePursuitController::continueMove(
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

    forward(relativeTargetPosition, relativeTargetVelocity, false);
}

bool PurePursuitController::completeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity)
{
    if (relativeTargetPosition->getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        forward(relativeTargetPosition, relativeTargetVelocity, true);
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    currentMovementState = MovementState::Turning;
    return true;
}

void PurePursuitController::forward(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity,
    bool completingMove)
{
    /*
    if (distanceToMove < DEFAULT_LOOK_AHEAD_DISTANCE) {
        double remainingLookAhead = DEFAULT_LOOK_AHEAD_DISTANCE - distanceToMove;
        ZVector2 actualTarget(
            relativeTargetPosition.getX() + relativeTargetVelocity.);
        actualTarget.set()
    }
    if (abs(currentMovement.orientation.get()) > M_PI_2) {
        //we're facing the wrong way
        zippy.turn(-currentMovement.orientation.get());
        return;
    }

    //adjust the movement depending on how well we will arrive at our target orientation
    double deltaOrientationAtTarget = subtractAngles(
        currentMovement.orientation.get(),
        2.0 * currentMovement.position.atan());
    if (abs(deltaOrientationAtTarget) > M_PI_2) {
        zippy.turn(-currentMovement.orientation.get());
        return;
    }
    */

    double factor = 1.0;
    if (!completingMove) {
        double deltaOrientationAtTarget = subtractAngles(
            relativeTargetVelocity->atan2(),
            2.0 * relativeTargetPosition->atan());
        factor = MIN_DELTA_TARGET_EPSILON + (MAX_DELTA_TARGET_EPSILON * cos(deltaOrientationAtTarget / 2.0));
    }

    if (relativeTargetPosition->getX() == 0.0) {
      zippy.moveLinear(relativeTargetPosition->getY() * factor);
      return;
    }

    double movementRadius = relativeTargetPosition->getD2() / (2.0 * relativeTargetPosition->getX());
    double movementTheta = 2.0 * relativeTargetPosition->atan();
    // double movementTheta = 2.0 * atanSafe(
        // -relativeTargetPosition->getY(),
        // (relativeTargetPosition->getX() - movementRadius));

    //calculations below derived from the following source
    //    http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm
    //
    //it should be noted that the signs of the radius and the angle herein use the following conventions...
    //    when the radius is negative, the robot moves around the circle defined by a center to the left of the front of the Zippy
    //    the sign of the angle indicates the direction the Zippy will turn such that a positive angle will always move around the
    //    circle in a clockwise direction; this leads to the following behaviors with respect to the combination of the signs of
    //    the radius and the angle
    //        +/+ moving forward to the right
    //        +/- moving backward to the right while the front of the Zippy is turning left
    //        -/- moving forward to the left
    //        -/+ moving backward to the left while the front of the Zippy is turning right
    // movementTheta *= factor;
    movementRadius *= factor;
    /*
    ZVector2 radiusToTarget = ZVector2(
        currentMovement.position.getY(),
        movementRadius - currentMovement.position.getX());
    double completeTurnToTarget = radiusToTarget.atan2();
    double movementTheta = completeTurnToTarget > 0.0
        ? completeTurnToTarget - acos(movementRadius / radiusToTarget.getD())
        : completeTurnToTarget + acos(movementRadius / radiusToTarget.getD());
    // */

    // SerialUSB.print("movement: ");
    // SerialUSB.print(movementRadius);
    // SerialUSB.print(", ");
    // SerialUSB.println(movementTheta);
    zippy.moveArc(movementRadius, movementTheta);
    // zippy.moveArc(movementRadius * factor, movementTheta);
    // zippy.moveArc(movementRadius, movementTheta);
}

void PurePursuitController::continueTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(relativeTargetPosition, relativeTargetVelocity))
                return;
            break;

        case MovementState::Stopped:
            zippy.start();
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;
            break;
    }

    zippy.turn(pad(relativeTargetVelocity->atan2(), ANGULAR_EPSILON));
}

bool PurePursuitController::completeTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    // if (abs(currentMovement.orientation.get()) > ANGULAR_EPSILON &&
        // stateDowngradeCounter)
    if (stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        zippy.turn(pad(relativeTargetVelocity->atan2(), ANGULAR_EPSILON));
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    currentMovementState = MovementState::Stopped;
    return true;
}

void PurePursuitController::completeStop(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(relativeTargetPosition, relativeTargetVelocity))
                return;

        case MovementState::Turning:
            if (!completeTurn(relativeTargetPosition, relativeTargetVelocity))
                return;
    }

    stop();
}

void PurePursuitController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
