
#include "zippies/pursuit/ScissorPursuitController.h"

void ScissorPursuitController::executeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    double linearTargetDistance = (relativeTargetPosition->getY() / 2.0) + (relativeTargetVelocity->getY() / 4.0);

    //first calculate the vector which represents the distance forward/backward to the midpoint of the velocity vector
    ZVector2 relativeEndPoint(
        relativeTargetPosition->getX() + (relativeTargetVelocity->getX() / 2.0),
        linearTargetDistance);
    /*
    if (reverseDirection) {
        relativeEndPoint.set(
            -linearTargetDistance * sin(relativeEndPoint.atan2()),
            linearTargetDistance - (linearTargetDistance * cos(relativeEndPoint.atan2())));
    }
    else {
    */
        relativeEndPoint.set(
            linearTargetDistance * sin(relativeEndPoint.atan2()),
            linearTargetDistance + (linearTargetDistance * cos(relativeEndPoint.atan2())));
    // }

    if (relativeEndPoint.getX() == 0.0) {
        linearVelocity = relativeEndPoint.getY();
        angularVelocity = 0.0;
        /*
        if (clipVelocities) {
            linearVelocity -= constrain(linearVelocity * CLIP_VELOCITY_FACTOR_MAX,
                -CLIP_VELOCITY_DISTANCE_MAX, CLIP_VELOCITY_DISTANCE_MAX);
        }
        */
    }
    else {
        angularVelocity = 2.0 * relativeEndPoint.atan();
        linearVelocity = angularVelocity * relativeEndPoint.getD2() / (2.0 * relativeEndPoint.getX());
        /*
        if (clipVelocities) {
            double clip = constrain(linearVelocity * CLIP_VELOCITY_FACTOR_MAX,
                -CLIP_VELOCITY_DISTANCE_MAX, CLIP_VELOCITY_DISTANCE_MAX);
            angularVelocity *= 1.0 - (clip / linearVelocity);
            linearVelocity -= clip;
        }
        */
    }
}
