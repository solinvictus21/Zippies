
#include "zippies/controllers/Driver.h"

#define DRIVER_PARAM_K1              1.2
#define DRIVER_PARAM_K2              2.0
#define DRIVER_PARAM_BETA            0.4
#define DRIVER_PARAM_LAMBDA          2.0

#define DRIVER_PARAM_MAX_LINEAR_ACCELERATION  10.0

double estimateDistance(const ZVector2* toRelativePosition, const ZRotation2* targetOrientation);

void Driver::reset()
{
    holdingPosition = false;
    reverseDirection = false;

    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;

    shadowPosition.reset();
    shadowVelocityVector.reset();
    shadowToTargetPosition.reset();

    previousTargetPosition.reset();
    targetPosition.reset();
}

void Driver::start(const ZMatrix2* sp, double tx, double ty, double to)
{
    shadowPosition.set(sp);
    previousTargetPosition.set(sp);
    targetPosition.set(tx, ty, to);
    holdingPosition = false;

    calculateNewTarget();
}

void Driver::start(double sx, double sy, double so, double tx, double ty, double to)
{
    shadowPosition.set(sx, sy, so);
    previousTargetPosition.set(sx, sy, so);
    targetPosition.set(tx, ty, to);
    holdingPosition = false;

    calculateNewTarget();
}

void Driver::setTargetPosition(double targetX, double targetY, double targetO)
{
    previousTargetPosition.set(&targetPosition);
    targetPosition.set(targetX, targetY, targetO);
    holdingPosition = false;

    calculateNewTarget();
}

void Driver::calculateNewTarget()
{
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());

    //calculate how much of a forward turn we would need to make to arrive at the new target endpoint
    /*
    double directionToTarget = atan2(
            targetPosition.position.getX() - previousTargetPosition.position.getX(),
            targetPosition.position.getY() - previousTargetPosition.position.getY());
    double directionFromPreviousOrientationToTarget = subtractAngles(
        directionToTarget, previousTargetPosition.position.atan2());
    double forwardTurn = addAngles(
        previousTargetPosition.orientation.get(),
        atan2(
            targetPosition.position.getX() - previousTargetPosition.position.getX(),
            targetPosition.position.getY() - previousTargetPosition.position.getY());
    double arrivalOrientation = addAngles(previousTargetPosition.orientation.get(), forwardTurn);
    */

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
    // SerialUSB.print("Reverse: ");
    // SerialUSB.println(reverseDirection);
}

void Driver::setTargetPosition(const ZMatrix2* tp)
{
    targetPosition.set(tp);
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
}

const double velocityFactor = 1000.0 / 60.0;

/* The following implementation is based on a research paper documented here...
 *        https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 *
 *    ...and the associated code example availabile here...
 *        https://github.com/h2ssh/Vulcan/blob/master/src/mpepc/control/kinematic_control_law.cpp
 */
void Driver::update(unsigned long remainingTime)
{
    if (holdingPosition)
        return;

    double distance = estimateDistance(&shadowToTargetPosition, &targetPosition.orientation);
    //this may be needed eventually to prevent "division by zero" halts from long-running routines; needs more analysis
    // if (distance == 0.0)
        // return;

    double lineOfSightOrientation = shadowToTargetPosition.atan2();

    double theta, delta;
    if (reverseDirection)
    {
        distance = -distance;
        theta = subtractAngles(addAngles(targetPosition.orientation.get(), M_PI), lineOfSightOrientation);
        delta = subtractAngles(addAngles(shadowPosition.orientation.get(), M_PI), lineOfSightOrientation);
    }
    else
    {
        theta = subtractAngles(targetPosition.orientation.get(), lineOfSightOrientation);
        delta = subtractAngles(shadowPosition.orientation.get(), lineOfSightOrientation);
    }

    double proportionalControlTerm = DRIVER_PARAM_K2 * subtractAngles(delta, atan(-DRIVER_PARAM_K1 * theta));
    double feedforwardControlTerm = sin(delta) * (1.0 + (DRIVER_PARAM_K1 / (1.0 + pow(DRIVER_PARAM_K1 * theta, 2.0))));

    //the value below represents the ratio between the number of radians that the robot should turn compared to the velocity
    //that the robot will move; as such, it is unitless and should not necessarily be used to calculate an appropriate
    //linear velocity
    double referenceKappa = distance == 0.0 ? 0.0 :
        -(proportionalControlTerm + feedforwardControlTerm) /
        distance;

    //choose a linear velocity based on our remaining distance to our current target point; note that velocity is represented
    //as millimeters per "frame" with a timing of 60 frames per second
    double idealLinearVelocity = remainingTime == 0 ? 0.0 : velocityFactor * distance / ((double)remainingTime);
    // double idealLinearAcceleration = idealLinearVelocity - currentLinearVelocity;

    //calculate the output linear and angular velocities, clipping acceleration to our predefined limits
    //currentLinearVelocity += ZMath.Clamp(idealLinearAcceleration,
    //-DRIVER_PARAM_MAX_LINEAR_ACCELERATION, DRIVER_PARAM_MAX_LINEAR_ACCELERATION);
    currentLinearVelocity = idealLinearVelocity / (1.0 + (DRIVER_PARAM_BETA * pow(abs(referenceKappa), DRIVER_PARAM_LAMBDA)));
    currentAngularVelocity = referenceKappa * currentLinearVelocity;

    if (currentAngularVelocity != 0.0)
    {
        double radius = currentLinearVelocity / currentAngularVelocity;
        shadowVelocityVector.set(
            radius - (radius * cos(currentAngularVelocity)),
            radius * sin(currentAngularVelocity));
        shadowVelocityVector.rotate(shadowPosition.orientation.get());
    }
    else
    {
        shadowVelocityVector.set(
            currentLinearVelocity * shadowPosition.orientation.sin(),
            currentLinearVelocity * shadowPosition.orientation.cos());
    }

    shadowPosition.position.add(&shadowVelocityVector);
    shadowPosition.orientation.add(currentAngularVelocity);

    /*
    SerialUSB.print(micros() / 1000);
    SerialUSB.print(", ");
    shadowPosition.printDebug();
    */

    //calculate the new position of our robot; this will eventually be used as the "current target position" on the actual robots
    shadowToTargetPosition.set(
        targetPosition.position.getX() - shadowPosition.position.getX(),
        targetPosition.position.getY() - shadowPosition.position.getY());
}

void Driver::holdPosition()
{
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    //move the shadow to the target
    shadowPosition.set(&targetPosition);
    // shadowVelocityVector.set(
        // shadowToTargetPosition.getX() * targetPosition.orientation.sin(),
        // shadowToTargetPosition.getY() * targetPosition.orientation.cos());
    // shadowVelocityVector.set(
        // shadowToTargetPosition.getD() * targetPosition.orientation.sin(),
        // shadowToTargetPosition.getD() * targetPosition.orientation.cos());
    shadowVelocityVector.set(targetPosition.orientation.sin(), targetPosition.orientation.cos());
    if (reverseDirection)
        shadowVelocityVector.flip();

    shadowToTargetPosition.reset();

    /*
    shadowPosition.printDebug();
    shadowVelocityVector.printDebug();
    // */

    holdingPosition = true;
}

double estimateDistance(const ZVector2* toRelativePosition, const ZRotation2* targetOrientation)
{
    return toRelativePosition->getD();
    /* //need to figure out how to calculate a decent length estimation
    double dotVector = abs((toRelativePosition->getX() * targetOrientation->sin()) +
        (toRelativePosition->getY() * targetOrientation->cos()));
    double crossVector = abs((toRelativePosition->getX() * targetOrientation->cos()) -
        (toRelativePosition->getY() * targetOrientation->sin()));

    double turnRadius = min(dotVector, crossVector);
    return (abs(toRelativePosition->atan()) * turnRadius) + toRelativePosition->getD() +
        (abs(subtractAngles(targetOrientation->get(), 2.0 * toRelativePosition->atan())));
    */
}
