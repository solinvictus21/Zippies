
#include "zippies/pursuit/ArcPursuitController.h"

void ArcPursuitController::continuePursuit(
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

    executeMove(currentPosition, targetPosition, targetVelocity);
}

void ArcPursuitController::stopPursuit(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    /*
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
    */

    stop();
}

void ArcPursuitController::executeMove(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    ZVector2 relativeTargetPosition(targetPosition);
    relativeTargetPosition.subtractVector(&currentPosition->position);
    relativeTargetPosition.unrotate(&currentPosition->orientation);
    ZVector2 relativeTargetVelocity(targetVelocity);
    relativeTargetVelocity.unrotate(&currentPosition->orientation);
    ZVector2 relativeEndPoint(&relativeTargetPosition);
    relativeEndPoint.addVector(&relativeTargetVelocity);

    /*
    double angleAtTargetToVelocity = subtractAngles(
        relativeTargetVelocity.atan2(),
        2.0 * relativeTargetPosition.atan());
        // snapAngle(2.0 * relativeTargetPosition.atan2()));
    if (abs(angleAtTargetToVelocity) > M_PI_2) {
        //just turn toward the target endpoint
        zippy.turn(relativeEndPoint.atan2() / 2.0);
        return;
    }
    */

    //Reference: https://discourse.mcneel.com/t/how-to-make-an-arc-meet-another-curve-perpendicular/32778/9
    double denominator = (2.0 * relativeTargetVelocity.getD()) + (2.0 * relativeEndPoint.getY());
    double moveY = (relativeEndPoint.getD2() - relativeTargetVelocity.getD2()) / denominator;
    relativeTargetPosition.set(relativeEndPoint.getX(), relativeEndPoint.getY() - moveY);
    if (denominator > 0.0)
        relativeTargetPosition.setD(moveY);
    else
        relativeTargetPosition.setD(-moveY);

    if (relativeTargetPosition.getX() == 0.0) {
        //just move forward or backward
        zippy.moveLinear(relativeTargetPosition.getY());
        return;
    }

    relativeTargetPosition.setY(relativeTargetPosition.getY() + moveY);
    double movementRadius = relativeTargetPosition.getD2() / (2.0 * relativeTargetPosition.getX());
    double movementTheta = 2.0 * relativeTargetPosition.atan();
    zippy.moveArc(movementRadius, movementTheta);
}

void ArcPursuitController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
