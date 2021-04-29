
#include "zippies/pursuit/ArcPursuitController.h"

void ArcPursuitController::continuePursuit(
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

    executeMove(relativeTargetPosition, relativeTargetVelocity);
}

bool ArcPursuitController::completeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    if (relativeTargetPosition->getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        executeMove(relativeTargetPosition, relativeTargetVelocity);
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    currentMovementState = MovementState::Turning;
    return true;
}

bool ArcPursuitController::completeTurn(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    if (abs(relativeTargetVelocity->atan2()) > ANGULAR_EPSILON &&
        stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        zippy.turn(relativeTargetVelocity->atan2());
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    zippy.stop();
    currentMovementState = MovementState::Stopped;
    return true;
}

void ArcPursuitController::stopPursuit(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove(relativeTargetPosition, relativeTargetVelocity))
                return;

        case MovementState::Turning:
            completeTurn(relativeTargetPosition, relativeTargetVelocity);
    }
}

void ArcPursuitController::executeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    ZVector2 relativeEndPoint(relativeTargetPosition);
    relativeEndPoint.addVector(relativeTargetVelocity);

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
    double denominator = (2.0 * relativeTargetVelocity->getD()) + (2.0 * relativeEndPoint.getY());
    if (denominator == 0.0) {
        zippy.moveLinear(relativeTargetPosition->getY());
        return;
    }

    double moveY = (relativeEndPoint.getD2() - relativeTargetVelocity->getD2()) / denominator;
    relativeEndPoint.setY(relativeEndPoint.getY() - moveY);
    if (denominator > 0.0)
        relativeEndPoint.setD(moveY);
    else
        relativeEndPoint.setD(-moveY);
    relativeEndPoint.setY(relativeEndPoint.getY() + moveY);

    if (relativeEndPoint.getX() == 0.0) {
        //just move forward or backward
        zippy.moveLinear(relativeEndPoint.getY());
        return;
    }

    double movementRadius = relativeEndPoint.getD2() / (2.0 * relativeEndPoint.getX());
    double movementTheta = 2.0 * relativeEndPoint.atan();
    zippy.moveArc(movementRadius, movementTheta);
}

void ArcPursuitController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
