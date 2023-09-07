
#include "zippies/controllers/Driver.h"

#define DRIVER_PARAM_K1              1.2
#define DRIVER_PARAM_K2              3.0
#define DRIVER_PARAM_BETA            0.4
#define DRIVER_PARAM_LAMBDA          2.0

#define DRIVER_PARAM_MAX_LINEAR_ACCELERATION  10.0

double estimateDistance(const ZVector2* toRelativePosition, double deltaOrientation);

void Driver::reset()
{
    remainingMovementTime = 0;
    reverseDirection = false;
    
    previousTargetPosition.reset();
    shadowPosition.reset();
    targetPosition.reset();
    shadowToTargetPosition.reset();
    
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    shadowVelocityVector.reset();
}

void Driver::start(const ZMatrix2* sp,
           double tx, double ty, double to,
           unsigned long movementTime)
{
    targetPosition.set(sp);
    shadowPosition.set(&targetPosition);
    remainingMovementTime = 0;

    setTargetPosition(tx, ty, to, movementTime);
}

void Driver::start(double sx, double sy, double so,
           double tx, double ty, double to,
           unsigned long movementTime)
{
    targetPosition.set(sx, sy, so);
    shadowPosition.set(&targetPosition);
    remainingMovementTime = 0;

    setTargetPosition(tx, ty, to, movementTime);
}

void Driver::setTargetPosition(const ZMatrix2* tp,
                       unsigned long movementTime)
{
    previousTargetPosition.set(&targetPosition);
    targetPosition.set(tp);
    remainingMovementTime += movementTime;

    calculateNewTarget();
}

void Driver::setTargetPosition(double tx, double ty, double to,
                       unsigned long movementTime)
{
    previousTargetPosition.set(&targetPosition);
    targetPosition.set(tx, ty, to);
    remainingMovementTime += movementTime;

    calculateNewTarget();
}

void Driver::calculateNewTarget()
{
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());

    double forwardTurn = 2.0 * subtractAngles(
        atan2(
            targetPosition.position.getX() - previousTargetPosition.position.getX(),
            targetPosition.position.getY() - previousTargetPosition.position.getY()),
        previousTargetPosition.orientation.get());
    //from our arrival orientation, add the difference to the next target orientation to calculate the total turn required
    forwardTurn += subtractAngles(targetPosition.orientation.get(),
        addAngles(previousTargetPosition.orientation.get(), forwardTurn));
    //if we need to do more than a 180 degree turn to arrive at the target orientation, then move backwards instead
    reverseDirection = abs(forwardTurn) > M_PI;
}

/* The following implementation is based on a research paper documented here...
 *        https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 *    ...and the associated code example availabile here...
 *        https://github.com/h2ssh/Vulcan/blob/master/src/mpepc/control/kinematic_control_law.cpp
 */
bool Driver::loop(unsigned long deltaTime)
{
    if (!remainingMovementTime)
        return true;
    
    if (deltaTime >= remainingMovementTime) {
        remainingMovementTime = 0;
        holdAtTarget();
        return true;
    }
    
    double lineOfSightOrientation = shadowToTargetPosition.atan2();
    double theta, delta, remainingDistance;
    if (reverseDirection)
    {
        theta = subtractAngles(addAngles(targetPosition.orientation.get(), M_PI), lineOfSightOrientation);
        delta = subtractAngles(addAngles(shadowPosition.orientation.get(), M_PI), lineOfSightOrientation);
        remainingDistance = -(delta == 0.0 ? fabs(shadowToTargetPosition.getY()) :
            (shadowToTargetPosition.getD() * delta) / sin(delta));
    }
    else
    {
        theta = subtractAngles(targetPosition.orientation.get(), lineOfSightOrientation);
        delta = subtractAngles(shadowPosition.orientation.get(), lineOfSightOrientation);
        remainingDistance = delta == 0.0 ? fabs(shadowToTargetPosition.getY()) :
            (shadowToTargetPosition.getD() * delta) / sin(delta);
    }
    
    //choose a linear velocity based on our remaining distance to our current target point; note that velocity is represented
    //as millimeters per "frame" with a timing of 60 frames per second
    double idealLinearVelocity = ((double)deltaTime) * remainingDistance / ((double)remainingMovementTime);
    remainingMovementTime -= deltaTime;

    double proportionalControlTerm = DRIVER_PARAM_K2 * subtractAngles(delta, atan(-DRIVER_PARAM_K1 * theta));
    double feedforwardControlTerm = sin(delta) * (1.0 + (DRIVER_PARAM_K1 / (1.0 + pow(DRIVER_PARAM_K1 * theta, 2.0))));

    //the value below represents the ratio between the number of radians that the robot should turn compared to the velocity
    //that the robot will move; as such, it is unitless and should not necessarily be used to calculate an appropriate
    //linear velocity
    double referenceKappa = remainingDistance == 0.0 ? 0.0 :
        -(proportionalControlTerm + feedforwardControlTerm) / fabs(remainingDistance);

    //calculate the output linear and angular velocities
    currentLinearVelocity = idealLinearVelocity / (1.0 + (DRIVER_PARAM_BETA * pow(abs(referenceKappa), DRIVER_PARAM_LAMBDA)));
    currentAngularVelocity = referenceKappa * currentLinearVelocity;

    if (currentAngularVelocity != 0.0) {
        double radius = currentLinearVelocity / currentAngularVelocity;
        shadowVelocityVector.set(
            radius - (radius * cos(currentAngularVelocity)),
            radius * sin(currentAngularVelocity));
        shadowVelocityVector.rotate(shadowPosition.orientation.get());
    }
    else {
        shadowVelocityVector.set(
            currentLinearVelocity * shadowPosition.orientation.sin(),
            currentLinearVelocity * shadowPosition.orientation.cos());
    }

    shadowPosition.position.add(&shadowVelocityVector);
    shadowPosition.orientation.add(currentAngularVelocity);

    //calculate the new position of our robot; this will eventually be used as the "current target position" on the actual robots
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
    return true;
}

void Driver::holdAtTarget()
{
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    //move the shadow to the target
    shadowPosition.set(&targetPosition);
    shadowVelocityVector.set(targetPosition.orientation.sin(), targetPosition.orientation.cos());
    reverseDirection = false;
    shadowToTargetPosition.reset();
}
