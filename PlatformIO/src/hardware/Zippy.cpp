
#include <SPI.h>
#include "zippies/hardware/Zippy.h"

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

void Zippy::turn(double orientation)
{
  motors.setMotors(
    leftWheel.turn(orientation),
    rightWheel.turn(orientation));
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
}
