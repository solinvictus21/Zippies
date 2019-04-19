
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "commands/QuadraticBezier1.h"
#include "commands/CubicBezier1.h"

#define WHEEL_OFFSET_X   16.7d
#define WHEEL_OFFSET_Y    5.9d
#define M_PI_34 2.356194490192345d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_POSITION_EPSILON                           30.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
#define ANGULAR_POSITION_EPSILON                  0.05235987755983d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       5000.00d

#define LINEAR_MAX_VELOCITY                      80.00d
#define ANGULAR_MAX_VELOCITY                     M_PI_4

Zippy::Zippy(unsigned long pui)
  : leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, pui),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y, pui)
{

#ifdef PLATFORM_TINYSCREEN
  face.displayFace();
#endif
}

void Zippy::move(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
}

void Zippy::turn(double orientation)
{
  currentTargetPosition.orientation = orientation;
  orientationUpdated = true;
}

//start all of our peripherals
void Zippy::start()
{
  motors.start();

  leftWheel.start();
  rightWheel.start();
}

bool Zippy::loop(const KPosition* currentPosition,
                 const KPosition* currentPositionDelta)
{
  KVector2 relativeVelocity(&currentPositionDelta->vector);
  double previousOrientation = subtractAngles(currentPosition->orientation, currentPositionDelta->orientation);
  relativeVelocity.rotate(-previousOrientation);

  KVector2 relativeTargetPosition(currentTargetPosition.vector.getX() - currentPosition->vector.getX(),
      currentTargetPosition.vector.getY() - currentPosition->vector.getY());
  relativeTargetPosition.rotate(-currentPosition->orientation);
  double relativeDirectionOfMotion = subtractAngles(currentTargetPosition.orientation, currentPosition->orientation);
  if (inReverse) {
    relativeTargetPosition.set(-relativeTargetPosition.getX(), -relativeTargetPosition.getY());
    relativeDirectionOfMotion = addAngles(relativeDirectionOfMotion, M_PI);
  }

  double linearVelocity, angularVelocity;
  if (!positionUpdated) {
    if (relativeTargetPosition.getD() < LINEAR_POSITION_EPSILON) {
      if (!orientationUpdated &&
        abs(relativeDirectionOfMotion) < ANGULAR_POSITION_EPSILON/* &&
        currentTargetVelocity < LINEAR_POSITION_EPSILON &&
        currentPositionDelta->vector.getD() < LINEAR_EPSILON &&
        abs(currentPositionDelta->orientation) < ANGULAR_POSITION_EPSILON*/)
      {
        //the position and orientation have stopped, we're in position and oriented and we're going slow enough to stop
        driveStop();
        return true;
      }

      //the position is not changing and we're near enough to it to just turn in place
      linearVelocity = 0.0d;
      angularVelocity = relativeDirectionOfMotion;
    }
    else {
      //drive directly to the point
      linearVelocity = min(relativeTargetPosition.getD(), LINEAR_MAX_VELOCITY);
      angularVelocity = relativeTargetPosition.getOrientation();
      if (abs(angularVelocity) > M_PI_2) {
        linearVelocity = -linearVelocity;
        angularVelocity = addAngles(angularVelocity, M_PI);
      }
    }
  }
  else
    plotBiArc(&relativeTargetPosition, relativeDirectionOfMotion, &linearVelocity, &angularVelocity);

  positionUpdated = false;
  orientationUpdated = false;

  if (inReverse) {
    linearVelocity = -linearVelocity;
    angularVelocity = addAngles(angularVelocity, M_PI);
  }

  driveArc(linearVelocity, angularVelocity);

  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
  left = saturate(left, LINEAR_MIN_POWER);
  right = saturate(right, LINEAR_MIN_POWER);
  setMotors(left, right);

  return false;
}

void Zippy::plotBiArc(
  KVector2* relativeTargetPosition, double relativeDirectionOfMotion,
  double* linearVelocity, double* angularVelocity)
{
  //derived from formulae and helpful explainations provided from the following site
  //  http://www.ryanjuckett.com/programming/biarc-interpolation/
  double cosO = cos(relativeDirectionOfMotion);
  double sinO = sin(relativeDirectionOfMotion);

  double d;
  if (abs(relativeDirectionOfMotion) == 0.0d) {
    double x = relativeTargetPosition->getX();
    double y = relativeTargetPosition->getY();
    if (y == 0.0d) {
      *linearVelocity = x / 2.0d;
      *angularVelocity = x > 0.0d ? M_PI : -M_PI;
      return;
    }

    //in this situation, our denominator becomes zero and d goes to infinity; handle this with different formulae
    double vDotT2 = (x * sinO) + (y * cosO);
    d = relativeTargetPosition->getD2() / (4.0d * vDotT2);
  }
  else {
    //t = t1 + t2
    //  where t1 = current relative orientation =  (0, 1)
    //        t2 = target relative orientation = (sin(o), cos(o))
    //v dot t = (v.x * t.x)           + (v.y * t.y)
    //        = (v.x * (t1.x * t2.x)) + (v.y * (t1.y + t2.y))
    //        = (v.x * sin(o))        + (v.y * (1 + cos(o)))
    double vDotT = (relativeTargetPosition->getX() * sinO) +
        (relativeTargetPosition->getY() * (1.0d + cosO));
    //t1 dot t2 = (t1.x * t2.x) + (t1.y * t2.y)
    //          = cos(o)
    //2 * (1 - (t1 dot t2)) = 2 * (1 - cos(o))
    double t1DotT2Inv2 = 2.0d * (1.0d - cosO);
    d = ( -vDotT + sqrt( pow(vDotT, 2.0d) + ( t1DotT2Inv2 * relativeTargetPosition->getD2() ) ) )
        / t1DotT2Inv2;
  }

  //pm = (p2 + (d * (t1 - t2))) / 2
  KVector2 pathConnectionPoint(-sinO, 1.0d - cosO);
  pathConnectionPoint.multiply(d);
  pathConnectionPoint.addVector(relativeTargetPosition);
  pathConnectionPoint.multiply(0.5d);

  *linearVelocity = pathConnectionPoint.getD();
  *angularVelocity = pathConnectionPoint.getOrientation();
}

void Zippy::driveArc(double linearVelocity, double angularVelocity)
{
  if (angularVelocity == 0.0d) {
    //move without turning
    leftWheel.moveStraight(linearVelocity);
    rightWheel.moveStraight(linearVelocity);
    return;
  }

  if (linearVelocity == 0.0d) {
    //turn without moving
    leftWheel.turn(angularVelocity);
    rightWheel.turn(angularVelocity);
    return;
  }

  double turnRadius = centerTurnRadius(linearVelocity, angularVelocity);
  leftWheel.move(
    turnRadius,
    angularVelocity);
  rightWheel.move(
    turnRadius,
    angularVelocity);
}

void Zippy::driveStop()
{
  // currentTargetVelocity = 0.0d;
  leftWheel.stop();
  rightWheel.stop();
  // setMotors(0.0d, 0.0d);
  motors.writeCommand(COMMAND_ALL_PWM, 30000, 30000, 30000, 30000);
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft > 0 ? motorLeft : 0,
      motorLeft < 0 ? -motorLeft : 0,
      motorRight > 0 ? motorRight : 0,
      motorRight < 0 ? -motorRight : 0);
}

double Zippy::centerTurnRadius(double distanceDelta, double orientationDelta)
{
  if (orientationDelta == 0.0d)
    return 0.0d;
  return distanceDelta / (2.0d * sin(-orientationDelta));
}

double Zippy::saturate(double a, double b)
{
  if (a == 0.0d)
    return 0.0d;

  return (a < 0.0d ? -b : b) + a;
}
