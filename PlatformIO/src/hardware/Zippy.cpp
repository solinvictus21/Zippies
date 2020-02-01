
#include <SPI.h>
#include "zippies/hardware/Zippy.h"
#include "zippies/config/MotorConfig.h"

Zippy::Zippy()
  : leftWheel(-WHEEL_CENTER_BODY_CENTER_OFFSET_X, -WHEEN_CENTER_BODY_CENTER_ANGLE),
    rightWheel(WHEEL_CENTER_BODY_CENTER_OFFSET_X, WHEEN_CENTER_BODY_CENTER_ANGLE),
    motors(MOTOR_DEAD_ZONE_ABS)
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
  if (target->position.getX() == 0.0d) {
    double targetY = target->position.getY();
    if (targetY == 0.0d)
      turn(target->orientation.get());
    else
      moveLinear(targetY);
    return;
  }

  double rC = target->position.getD() / (2.0d * sin(target->position.atan2()));
  double theta = 2.0d * target->position.atan();
  leftWheel.moveArc(rC, theta);
  rightWheel.moveArc(rC, theta);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::moveLinear(double targetLinearVelocity)
{
  leftWheel.moveStraight(targetLinearVelocity);
  rightWheel.moveStraight(targetLinearVelocity);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::turn(double orientation)
{
  leftWheel.turn(orientation);
  rightWheel.turn(orientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

//stopped when the signal from the lighthouse is lost
void Zippy::stop()
{
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
}
