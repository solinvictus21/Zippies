
#include "zippies/hardware/Zippy.h"

#ifdef WEBOTS_SUPPORT
Zippy::Zippy(Robot* zippyWebots)
  : motors(zippyWebots),
#else
Zippy::Zippy()
  : motors(),
#endif
    leftWheel(-WHEEL_CENTER_BODY_CENTER_OFFSET),
    rightWheel(WHEEL_CENTER_BODY_CENTER_OFFSET)
{
}

void Zippy::start()
{
    leftWheel.start();
    rightWheel.start();
}

void Zippy::setInput(const ZMatrix2* positionDelta)
{
    double linearVelocity =
        (positionDelta->position.getD() * positionDelta->position.atan()) /
        sin(positionDelta->position.atan());
    setInput(linearVelocity, positionDelta->orientation.get());
}

void Zippy::setInput(double linearVelocity, double angularVelocity)
{
    if (angularVelocity == 0.0) {
        leftWheel.setInput(linearVelocity);
        rightWheel.setInput(linearVelocity);
        return;
    }

    double radius = linearVelocity / angularVelocity;
    leftWheel.setInput((radius + WHEEL_CENTER_BODY_CENTER_OFFSET) * angularVelocity);
    rightWheel.setInput((radius - WHEEL_CENTER_BODY_CENTER_OFFSET) * angularVelocity);
}

void Zippy::moveLinear(double targetLinearVelocity)
{
    motors.setMotors(
        leftWheel.moveStraight(targetLinearVelocity),
        rightWheel.moveStraight(targetLinearVelocity));
}

void Zippy::move(double linearVelocity, double angularVelocity)
{
    if (angularVelocity == 0.0) {
        motors.setMotors(
            leftWheel.moveStraight(linearVelocity),
            rightWheel.moveStraight(linearVelocity));
        return;
    }

    double radius = linearVelocity / angularVelocity;
    motors.setMotors(
        leftWheel.moveArc(radius, angularVelocity),
        rightWheel.moveArc(radius, angularVelocity));
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
    leftWheel.stop();
    rightWheel.stop();
    motors.stopMotors();
}
