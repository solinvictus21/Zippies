
#include "zippies/pursuit/StanleyPursuitController.h"

#define STANLEY_KV                    1.0

void StanleyPursuitController::executeMove(
    const ZVector2* relativeTargetPosition,
    const ZVector2* relativeTargetVelocity)
{
    if (relativeTargetVelocity->getD() == 0.0) {
        linearVelocity = 0.0;
        angularVelocity = 0.0;
        return;
    }

    //calculate position error
    double crossProduct =
        ((relativeTargetPosition->getX() * relativeTargetVelocity->getY()) -
        (relativeTargetPosition->getY() * relativeTargetVelocity->getX())) /
        relativeTargetVelocity->getD();
    double dotProduct = 
        ((relativeTargetPosition->getX() * relativeTargetVelocity->getX()) +
        (relativeTargetPosition->getY() * relativeTargetVelocity->getY())) /
        relativeTargetVelocity->getD();

    linearVelocity = relativeTargetVelocity->getD() + dotProduct;
    angularVelocity = addAngles(relativeTargetVelocity->atan2(), atanSafe(STANLEY_KV * crossProduct, linearVelocity));

    // linearVelocity *= 1.0 - abs(angularVelocity / M_PI);
}
