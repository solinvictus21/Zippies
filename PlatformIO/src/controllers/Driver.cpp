
#include "zippies/controllers/Driver.h"

#define DRIVER_PARAM_K1              1.2
#define DRIVER_PARAM_K2              2.0

#define DRIVER_PARAM_MAX_LINEAR_ACCELERATION  5.0

void Driver::reset()
{
    reverseDirection = false;
    shadowVelocityVector.reset();

    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;

    shadowPosition.reset();
    targetPosition.reset();
    shadowToTargetPosition.reset();
}

/*
void Driver::setAnchorPosition(double anchorX, double anchorY, double anchorO)
{
    reverseDirection = false;
    anchorPosition.set(anchorX, anchorY, anchorO);
    shadowVelocityVector.reset();

    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;

    shadowPosition.set(&anchorPosition);
    targetPosition.set(&anchorPosition);
    shadowToTargetPosition.reset();
}
*/

void Driver::setShadowPosition(const ZMatrix2* sp)
{
    shadowPosition.set(sp);
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
}

void Driver::setShadowPosition(double sx, double sy, double so)
{
    shadowPosition.set(sx, sy, so);
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
}

void Driver::setTargetPosition(double targetX, double targetY, double targetO)
{
    targetPosition.set(targetX, targetY, targetO);
    // targetPosition.concat(&anchorPosition);
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
    // SerialUSB.print("Setting new target position: ");
    // targetPosition.printDebug();
}

void Driver::setTargetPosition(const ZMatrix2* tp)
{
    targetPosition.set(tp);
    // targetPosition.concat(&anchorPosition);
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
    // SerialUSB.print("Setting new target position: ");
    // targetPosition.printDebug();
}

void Driver::update(unsigned long remainingTime)
{
    const double velocityFactor = 1000.0 / 60.0;

    //The following implementation is based on a research paper documented at the following location...
    //
    //      https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
    //      https://github.com/h2ssh/Vulcan/blob/master/src/mpepc/control/kinematic_control_law.cpp
    //
    double lineOfSightOrientation = shadowToTargetPosition.atan2();
    double distance = shadowToTargetPosition.getD();
    double theta, delta;
    if (reverseDirection) {
        distance = -distance;
        theta = subtractAngles(addAngles(targetPosition.orientation.get(), M_PI), lineOfSightOrientation);
        delta = subtractAngles(addAngles(shadowPosition.orientation.get(), M_PI), lineOfSightOrientation);
    }
    else {
        theta = subtractAngles(targetPosition.orientation.get(), lineOfSightOrientation);
        delta = subtractAngles(shadowPosition.orientation.get(), lineOfSightOrientation);
    }

    double proportionalControlTerm = DRIVER_PARAM_K2 * subtractAngles(delta, atan(-DRIVER_PARAM_K1 * theta));
    double feedforwardControlTerm = sin(delta) * (1.0 + (DRIVER_PARAM_K1 / (1.0 + pow(DRIVER_PARAM_K1 * theta, 2.0))));
    double referenceKappa =
        -(proportionalControlTerm + feedforwardControlTerm) /
        distance;

    double idealLinearVelocity = velocityFactor * distance / ((double)remainingTime);
    double idealLinearAcceleration = idealLinearVelocity - currentLinearVelocity;

    //calculate the velocities
    currentLinearVelocity += constrain(idealLinearAcceleration,
        -DRIVER_PARAM_MAX_LINEAR_ACCELERATION, DRIVER_PARAM_MAX_LINEAR_ACCELERATION);
    // SerialUSB.println(currentLinearVelocity);
    currentAngularVelocity = referenceKappa * currentLinearVelocity;
    // SerialUSB.print("LV: ");
    // SerialUSB.println(currentLinearVelocity);
    if (currentAngularVelocity != 0.0) {
        double radius = currentLinearVelocity / currentAngularVelocity;
        shadowVelocityVector.set(
            radius - (radius * cos(currentAngularVelocity)),
            radius * sin(currentAngularVelocity));
    }
    else
        shadowVelocityVector.set(0.0, currentLinearVelocity);        
    shadowPosition.append(&shadowVelocityVector);
    // SerialUSB.print("  Time: ");
    // SerialUSB.print(remainingTime);
    // SerialUSB.print("  Shadow: ");
    // shadowPosition.printDebug();
    // relativeShadowVelocity.unrotate(&sensors->getPosition()->orientation);

    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
}

void Driver::stop()
{
    currentLinearVelocity = 1.0;
    currentAngularVelocity = 0.0;
    //move the shadow to the target
    shadowPosition.set(&targetPosition);
    shadowToTargetPosition.reset();
    shadowVelocityVector.set(
        targetPosition.orientation.sin(), targetPosition.orientation.cos());
}