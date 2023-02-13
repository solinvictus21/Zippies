
#include <SPI.h>
#include "zippies/hardware/Zippy.h"

// #define LIMIT_ACCELERATION 1
#ifdef LIMIT_ACCELERATION

#define LINEAR_ACCELERATION_MAX                                2.0d
// #define ANGULAR_ACCELERATION_MAX                               0.01d
// #define ANGULAR_ACCELERATION_MAX                               0.052359877559830d  //3 degrees
// #define ANGULAR_ACCELERATION_MAX                               0.069813170079773d  //4 degrees
// #define ANGULAR_ACCELERATION_MAX                               0.087266462599716d  //5 degrees
#define ANGULAR_ACCELERATION_MAX                               0.104719755119660d  //6 degrees
// #define ANGULAR_ACCELERATION_MAX                               0.139626340159546d  //8 degrees
// #define ANGULAR_ACCELERATION_MAX                               0.174532925199433d  //10 degrees
// #define ANGULAR_ACCELERATION_MAX                               0.261799387799149d  //15 degrees
// #define ANGULAR_ACCELERATION_MAX                               M_PI

#endif

Zippy::Zippy()
  : leftWheel( -WHEEL_CENTER_BODY_CENTER_OFFSET_X, -WHEEL_CENTER_BODY_CENTER_OFFSET_Y),
    rightWheel( WHEEL_CENTER_BODY_CENTER_OFFSET_X,  WHEEL_CENTER_BODY_CENTER_OFFSET_Y)
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
    leftWheel.setInput((radius + WHEEL_CENTER_BODY_CENTER_OFFSET_X) * angularVelocity);
    rightWheel.setInput((radius - WHEEL_CENTER_BODY_CENTER_OFFSET_X) * angularVelocity);
}

void Zippy::moveLinear(double targetLinearVelocity)
{
    motors.setMotors(
        leftWheel.moveStraight(targetLinearVelocity),
        rightWheel.moveStraight(targetLinearVelocity));
}

void Zippy::moveArc(double radius, double theta)
{
    motors.setMotors(
        leftWheel.moveArc(radius, theta),
        rightWheel.moveArc(radius, theta));
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
    /*
    double left = leftWheel.moveArc(radius, angularVelocity);
    double right = rightWheel.moveArc(radius, angularVelocity);
    SerialUSB.print("lv: ");
    SerialUSB.print(linearVelocity);
    SerialUSB.print("   av: ");
    SerialUSB.print(angularVelocity);
    SerialUSB.print("   left: ");
    SerialUSB.print(left);
    SerialUSB.print("  right: ");
    SerialUSB.println(right);
    motors.setMotors(left, right);
    */
    motors.setMotors(
        leftWheel.moveArc(radius, angularVelocity),
        rightWheel.moveArc(radius, angularVelocity));
}

void Zippy::turn(double yOffset, double angularVelocity)
{
    motors.setMotors(
        leftWheel.moveArc(0.0, angularVelocity),
        rightWheel.moveArc(0.0, angularVelocity));
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
    leftWheel.stop();
    rightWheel.stop();
    motors.stopMotors();
}
