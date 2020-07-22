
#include "zippies/controllers/PathFollowingController.h"
#include <Arduino.h>

// #define LINEAR_EPSILON                               10.00  //1cm
#define LINEAR_EPSILON                               15.00  //1.5cm
// #define LINEAR_EPSILON                               20.00  //2cm
// #define LINEAR_EPSILON                               30.00  //3cm

#define ANGULAR_EPSILON                               0.052359877559830  //3 degrees
// #define ANGULAR_EPSILON                               0.069813170079773  //4 degrees
// #define ANGULAR_EPSILON                               0.087266462599716  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149  //15 degrees

#define MAX_STATE_DOWNGRADE_ITERATIONS               30
// #define MAX_STATE_DOWNGRADE_ITERATIONS              120

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

void PathFollowingController::followPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    currentMovement.position.set(targetPosition);
    currentMovement.orientation.set(targetVelocity->atan2());
    currentMovement.unconcat(currentPosition);
    currentVelocityTarget.set(
        targetPosition->getX() + (targetVelocity->getX() / 60.0),
        targetPosition->getY() + (targetVelocity->getY() / 60.0));
    currentVelocityTarget.subtractVector(&currentPosition->position);
    currentVelocityTarget.rotate(currentPosition->orientation.get());

    /*
    pluckerS.set(targetVelocity);
    pluckerS.subtractVector(&currentPosition->position);
    pluckerS.rotate(&currentPosition->orientation);
    pluckerS.normalize();
    double rotation =
        (currentMovement.position.getX() * pluckerS.getY()) -
        (currentMovement.position.getY() * pluckerS.getX());
    */

    continueMove();
}

void PathFollowingController::stopPath(
    const ZMatrix2* currentPosition,
    const ZVector2* targetPosition,
    const ZVector2* targetVelocity)
{
    currentMovement.set(
        targetPosition->getX(), targetPosition->getY(),
        targetVelocity->atan2());
    currentMovement.unconcat(currentPosition);
    // double positionDotDirection =
        // (currentMovement.position.getX() * currentMovement.orientation.sin()) +
        // (currentMovement.position.getX() * currentMovement.orientation.cos());
    completeStop();
}

void PathFollowingController::continueMove()
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

    forward(false);
}

bool PathFollowingController::completeMove()
{
    if (currentMovement.position.getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        forward(true);
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    currentMovementState = MovementState::Turning;
    return true;
}

void PathFollowingController::forward(bool completingMove)
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
            currentMovement.orientation.get(),
            2.0 * currentMovement.position.atan());
        factor = MIN_DELTA_TARGET_EPSILON + (MAX_DELTA_TARGET_EPSILON * cos(deltaOrientationAtTarget / 2.0));
    }

    if (currentMovement.position.getX() == 0.0) {
      zippy.moveLinear(currentMovement.position.getY() * factor);
      return;
    }

    double movementRadius = currentVelocityTarget.getD2() / (2.0 * currentVelocityTarget.getX());
    double movementTheta = 2.0 * atanSafe(
        -currentMovement.position.getY(),
        (currentMovement.position.getX() - movementRadius));

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
    // double movementRadius = currentMovement.position.getD2() / (2.0 * currentMovement.position.getX());
    // double movementTheta = snapAngle(2.0 * currentMovement.position.atan2());
    // double movementTheta = 2.0 * currentMovement.position.atan();
    // movementTheta *= factor;
    // movementTheta += subtractAngles(-snapAngle(2.0 * currentMovement.orientation.get()), movementTheta) * (1.0 - factor);
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

void PathFollowingController::continueTurn()
{
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove())
                return;
            break;

        case MovementState::Stopped:
            zippy.start();
            stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
            currentMovementState = MovementState::Turning;
            break;
    }

    zippy.turn(pad(currentMovement.orientation.get(), ANGULAR_EPSILON));
}

bool PathFollowingController::completeTurn()
{
    // if (abs(currentMovement.orientation.get()) > ANGULAR_EPSILON &&
        // stateDowngradeCounter)
    if (stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        zippy.turn(pad(currentMovement.orientation.get(), ANGULAR_EPSILON));
        return false;
    }

    stateDowngradeCounter = MAX_STATE_DOWNGRADE_ITERATIONS;
    currentMovementState = MovementState::Stopped;
    return true;
}

void PathFollowingController::completeStop()
{
    switch (currentMovementState) {
        case MovementState::Moving:
            if (!completeMove())
                return;

        case MovementState::Turning:
            if (!completeTurn())
                return;
    }

    stop();
}

void PathFollowingController::stop()
{
    zippy.stop();
    currentMovementState = MovementState::Stopped;
}
