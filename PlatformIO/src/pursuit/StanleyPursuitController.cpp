
#include "zippies/pursuit/StanleyPursuitController.h"

#define STANLEY_KV                    1.5

void StanleyPursuitController::executeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    double distanceTraveled = relativeTargetVelocity->getD();
    if (distanceTraveled == 0.0) {
        linearVelocity = 0.0;
        angularVelocity = 2.0 * relativeTargetVelocity->atan();
        return;
    }

    double dotProduct = 
        ((relativeTargetPosition->getX() * relativeTargetVelocity->getX()) +
        (relativeTargetPosition->getY() * relativeTargetVelocity->getY())) /
        distanceTraveled;
    linearVelocity = dotProduct;

    double crossProduct =
        ((relativeTargetPosition->getX() * relativeTargetVelocity->getY()) -
        (relativeTargetPosition->getY() * relativeTargetVelocity->getX())) /
        distanceTraveled;
    angularVelocity = 2.0 * addAngles(relativeTargetVelocity->atan2(), atanSafe(STANLEY_KV * crossProduct, linearVelocity));
}
