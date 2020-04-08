
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

//calculations below derived from the following source
//    http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm
//it should be noted that the signs of the radius and the angle herein use the following conventions
//    when the radius is negative, the robot moves around the circle defined by a center to the left of the front of the Zippy
//    the sign of the angle indicates the direction the Zippy will turn; thus...
//        when the radius is positive, a positive angle will move around the circle clockwise
//        when the radius is negative, a positive angle will move around the circle clockwise
//    this leads to the following behaviors with respect to the combination of the signs of the radius and the angle
//        +/+ moving forward to the right
//        +/- moving backward to the right while the front of the Zippy is turning left
//        -/- moving forward to the left
//        -/+ moving backward to the left while the front of the Zippy is turning right
void Zippy::move(const KMatrix2* target)
{
#ifdef LIMIT_ACCELERATION
  double newTargetLinearVelocity = target->position.getD();
  double newTargetAngularVelocity;
  if (newTargetLinearVelocity != 0.0d) {
    newTargetAngularVelocity = target->position.atan();
    if (newTargetAngularVelocity != 0.0d)
      newTargetLinearVelocity *= newTargetAngularVelocity / sin(newTargetAngularVelocity);
    newTargetAngularVelocity *= 2.0d;
  }
  else
    newTargetAngularVelocity = target->orientation.get();
  setTargetVelocities(newTargetLinearVelocity, newTargetAngularVelocity);
#else
  if (target->position.getX() == 0.0d) {
    double targetY = target->position.getY();
    if (targetY == 0.0d)
      turn(target->orientation.get());
    else
      moveLinear(targetY);
    return;
  }

  // double rC = target->position.getD() / (2.0d * sin(target->position.atan2()));
  double rC = target->position.getD2() / (2.0d * target->position.getX());
  double theta = 2.0d * target->position.atan();
  // leftWheel.moveArc(rC, theta);
  // rightWheel.moveArc(rC, theta);
  // motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
  motors.setMotors(
    leftWheel.moveArc(rC, theta),
    rightWheel.moveArc(rC, theta));
#endif
}

void Zippy::moveLinear(double targetLinearVelocity)
{
  // leftWheel.moveStraight(targetLinearVelocity);
  // rightWheel.moveStraight(targetLinearVelocity);
  // motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
  motors.setMotors(
    leftWheel.moveStraight(targetLinearVelocity),
    rightWheel.moveStraight(targetLinearVelocity));
}

/*
void Zippy::turn(double radius, double orientation)
{
#ifdef LIMIT_ACCELERATION
  setTargetVelocities(0.0d, orientation);
#else
  leftWheel.moveArc(radius, orientation);
  rightWheel.moveArc(radius, orientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
#endif
}
*/

void Zippy::moveArc(double radius, double theta)
{
  motors.setMotors(
    leftWheel.moveArc(radius, theta),
    rightWheel.moveArc(radius, theta));
}

void Zippy::turn(double orientation)
{
#ifdef LIMIT_ACCELERATION
  setTargetVelocities(0.0d, orientation);
#else
  /*
  leftWheel.turn(orientation);
  rightWheel.turn(orientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
  */
  motors.setMotors(
    leftWheel.turn(orientation),
    rightWheel.turn(orientation));
#endif
}

#ifdef LIMIT_ACCELERATION
void Zippy::setTargetVelocities(double targetLinearVelocity, double targetAngularVelocity)
{
  // this->targetLinearVelocity = targetLinearVelocity;
  // targetLinearVelocity = constrain(targetLinearVelocity, -30.0d, 30.0d);
  linearAcceleration = constrain(targetLinearVelocity - linearVelocity,
      -LINEAR_ACCELERATION_MAX, LINEAR_ACCELERATION_MAX);
  // linearVelocity += linearAcceleration;
  linearVelocity += linearAcceleration;

  // this->targetAngularVelocity = targetAngularVelocity;
  targetAngularVelocity = constrain(targetAngularVelocity, -M_PI_2, M_PI_2);
  // angularAcceleration = targetAngularVelocity - angularVelocity;
  angularAcceleration = constrain(angularAcceleration + (targetAngularVelocity - angularVelocity),
      -ANGULAR_ACCELERATION_MAX, ANGULAR_ACCELERATION_MAX);
  angularVelocity += angularAcceleration;
  // angularAcceleration += constrain(angularAcceleration + constrain(targetAngularAcceleration - angularAcceleration,
      // -ANGULAR_ACCELERATION_MAX, ANGULAR_ACCELERATION_MAX),
      // -0.5d, 0.5d);
  // angularVelocity = constrain(angularVelocity + angularAcceleration, -1.0d, 1.0d);

  if (angularVelocity == 0.0d) {
    if (linearVelocity == 0.0d) {
      // leftWheel.moveStraight(0.0d);
      // rightWheel.moveStraight(0.0d);
      motors.setMotors(
        leftWheel.moveStraight(0.0d),
        rightWheel.moveStraight(0.0d));
    }
    else {
      // leftWheel.moveStraight(linearVelocity);
      // rightWheel.moveStraight(linearVelocity);
      motors.setMotors(
        leftWheel.moveStraight(linearVelocity),
        rightWheel.moveStraight(linearVelocity));
    }
  }
  else if (linearVelocity == 0.0d) {
    // leftWheel.turn(angularVelocity);
    // rightWheel.turn(angularVelocity);
    motors.setMotors(
      leftWheel.turn(angularVelocity),
      rightWheel.turn(angularVelocity));
  }
  else {
    double rC = abs(linearVelocity) / (2.0d * sin(angularVelocity));
    // leftWheel.moveArc(rC, angularVelocity);
    // rightWheel.moveArc(rC, angularVelocity);
    motors.setMotors(
      leftWheel.moveArc(rC, angularVelocity),
      rightWheel.moveArc(rC, angularVelocity));
  }

  // motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}
#endif

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
#ifdef LIMIT_ACCELERATION
  linearVelocity = 0.0d;
  linearAcceleration = 0.0d;
  angularVelocity = 0.0d;
  angularAcceleration = 0.0d;
#endif
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
}
