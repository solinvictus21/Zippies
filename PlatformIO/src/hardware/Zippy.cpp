
#include "zippies/hardware/Zippy.h"
#include "zippies/config/BodyConfig.h"

#ifdef WEBOTS_SUPPORT
Zippy::Zippy(const ZippyConfig* zippyConfig, Robot* zippyWebots)
  : motors(zippyConfig, zippyWebots),
#else
Zippy::Zippy(const ZippyConfig* zc)
  : motors(zc),
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
    double angularVelocity = positionDelta->position.atan();
    if (angularVelocity == 0.0) {
        double linearVelocity = positionDelta->position.getY();
        leftWheel.setInput(linearVelocity);
        rightWheel.setInput(linearVelocity);
        return;
    }
    
    double linearVelocity = (positionDelta->position.getD() * positionDelta->position.atan()) /
        sin(positionDelta->position.atan());
    if (positionDelta->position.getY() < 0.0)
        linearVelocity = -linearVelocity;

    double radius = linearVelocity / angularVelocity;
    leftWheel.setInput((radius + WHEEL_CENTER_BODY_CENTER_OFFSET) * angularVelocity);
    rightWheel.setInput((radius - WHEEL_CENTER_BODY_CENTER_OFFSET) * angularVelocity);
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
