
#include <SPI.h>
#include "zippies/hardware/Zippy.h"
#include "zippies/config/MotorConfig.h"

#define TARGET_EPSILON                                0.20d

#define LINEAR_EPSILON                                8.00d
// #define LINEAR_EPSILON                               10.00d
const double MIN_LINEAR_EPSILON = LINEAR_EPSILON * (1.0d - TARGET_EPSILON);
const double MAX_LINEAR_EPSILON = LINEAR_EPSILON * (1.0d + TARGET_EPSILON);

// #define ANGULAR_EPSILON                               0.052359877559830d  //3 degrees
// #define ANGULAR_EPSILON                               0.087266462599716d  //5 degrees
#define ANGULAR_EPSILON                               0.104719755119660d  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546d  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433d  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149d  //15 degrees
// #define ANGULAR_EPSILON                               0.069813170079773d  //4 degrees
const double MIN_ANGULAR_EPSILON = ANGULAR_EPSILON * (1.0d - TARGET_EPSILON);
const double MAX_ANGULAR_EPSILON = ANGULAR_EPSILON * (1.0d + TARGET_EPSILON);

//the distince in mm within which is to be considered "at the target" for the purpose of terminating movement
#define FOLLOW_FACTOR                                 0.50d
//minimum angular movement

// #define WHEEL_RADIAL_OFFSET       17.711578134090706d
// #define WHEEL_ANGLE_OFFSET        19.457979011589846d

#define MAX_ACCELERATION                              50.0d

Zippy::Zippy()
  // : leftWheel(-WHEEL_CENTER_BODY_CENTER_OFFSET, -WHEEN_CENTER_BODY_CENTER_ANGLE),
    // rightWheel(WHEEL_CENTER_BODY_CENTER_OFFSET, WHEEN_CENTER_BODY_CENTER_ANGLE),
  : leftWheel(-WHEEL_CENTER_BODY_CENTER_OFFSET_X, -WHEEN_CENTER_BODY_CENTER_ANGLE),
    rightWheel(WHEEL_CENTER_BODY_CENTER_OFFSET_X, WHEEN_CENTER_BODY_CENTER_ANGLE),
    motors(MOTOR_DEAD_ZONE_ABS)
{
}

void Zippy::start()
{
}

void Zippy::startErrorCapture()
{
  errorCaptureEnabled = true;
  statisticsAccumulator.reset();
}

void Zippy::setCurrentPosition(const KMatrix2* p)
{
  currentPosition.set(p);

  //only capture error statistics while moving
  if (errorCaptureEnabled && currentMovementState == MovementState::Moving) {
    //calculate error based on how close we came to our target position
    double error = sqrt(
        sq(currentPosition.position.getX() - targetPosition.position.getX()) +
        sq(currentPosition.position.getY() - targetPosition.position.getY()));
    statisticsAccumulator.accumulate(error);
  }
}

double Zippy::getStandardDeviation() const
{
  return statisticsAccumulator.getStandardDeviation();
}

void Zippy::setTargetPosition(const KMatrix2* tp)
{
  targetPosition.set(tp);
  targetPositionUpdated = true;
  targetOrientationUpdated = true;
}

void Zippy::setTargetOrientation(const KRotation2* r)
{
  targetPosition.orientation.set(r);
  targetOrientationUpdated = true;
  if (currentMovementState == MovementState::Stopped) {
    leftWheel.start();
    rightWheel.start();
    currentMovementState = MovementState::Turning;
  }
}

void Zippy::loop()
{
  //check if we need to upgrade or downgrade our current movement state
  KMatrix2 targetVelocity(&targetPosition);
  targetVelocity.unconcat(&currentPosition);
  switch (currentMovementState) {
    case MovementState::Moving:
      if (!targetPositionUpdated && targetVelocity.position.getD() < MIN_LINEAR_EPSILON)
        currentMovementState = MovementState::Turning;
      break;

    case MovementState::Turning:
      if (targetVelocity.position.getD() >= MAX_LINEAR_EPSILON)
        currentMovementState = MovementState::Moving;
      else if (!targetOrientationUpdated && abs(targetVelocity.orientation.get()) < MIN_ANGULAR_EPSILON)
        stop();
      break;

    case MovementState::Stopped:
      if (targetVelocity.position.getD() >= MAX_LINEAR_EPSILON) {
        leftWheel.start();
        rightWheel.start();
        currentMovementState = MovementState::Moving;
      }
      else if (abs(targetVelocity.orientation.get()) >= MAX_ANGULAR_EPSILON) {
        leftWheel.start();
        rightWheel.start();
        currentMovementState = MovementState::Turning;
      }
      break;
  }

  //now execute a move if needed
  switch (currentMovementState) {
    case MovementState::Moving:
      executeMove(&targetVelocity);
      break;

    case MovementState::Turning:
      executeTurn(targetVelocity.orientation.get());
      break;
  }

  targetPositionUpdated = false;
  targetOrientationUpdated = false;
}

//execute a move, possibly with a turn, forming an arc
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
void Zippy::executeMove(KMatrix2* targetVelocity)
{
  double relativeThetaG = targetVelocity->position.atan();
  if (relativeThetaG == 0.0d) {
    //no turn, so this just becomes a linear move; this is an alternate code path because the calculation to move along an
    //arc has an asymptote at zero such that the radius goes to infinity
    double targetLinearVelocity = FOLLOW_FACTOR * targetVelocity->position.getY();
    /*
    targetLinearVelocity = targetLinearVelocity > 0.0d
        ? max(0.0d, targetLinearVelocity - MIN_LINEAR_EPSILON)
        : min(0.0d, targetLinearVelocity + MIN_LINEAR_EPSILON);
    */
    if (errorCaptureEnabled) {
      targetVelocity->position.set(0.0d, targetLinearVelocity);
      targetPosition.set(targetVelocity);
      targetPosition.concat(&currentPosition);
    }

    executeLinearMove(targetLinearVelocity);
    return;
  }

  //from documentation reference, equation [3]
  double relativeTheta = 2.0d * relativeThetaG;

  // /*
  //follow behind the target point while actively Moving
  // if (targetPositionUpdated) {
    /*
    if (abs(subtractAngles(subtendedAngle, targetVelocity->orientation.get())) >= M_PI_2) {
      //in this situation, moving toward the point faces us away from the direction the target is moving
      //in this scenario, we're just going to slow down and turn toward the next hypothetical point based on our current velocity
      targetVelocity->position.set(
          targetVelocity->position.getX() + (targetLinearVelocity * sin(targetVelocity->orientation.get())),
          targetVelocity->position.getY() + (targetLinearVelocity * cos(targetVelocity->orientation.get())));
      targetAtan = targetVelocity->position.atan();
      // subtendedAngle = 2.0d * targetAtan;
      //intentional; not a bug; only move through half the arc, since we passed the target
      subtendedAngle = targetAtan;
    }
    */

    relativeTheta *= FOLLOW_FACTOR;
  // }
  // */


  //simple arc; from documentation reference, equation [4]
  double rC = targetVelocity->position.getD() / (2.0 * sin(targetVelocity->position.atan2()));
  /*
  double idealLinearVelocity = rC * relativeTheta;
  double targetLinearVelocity = idealLinearVelocity > 0.0d
      ? max(0.0d, idealLinearVelocity - MIN_LINEAR_EPSILON)
      : min(0.0d, idealLinearVelocity + MIN_LINEAR_EPSILON);
  relativeTheta *= targetLinearVelocity / idealLinearVelocity;
  // */
  /*
  double newTargetLinearVelocity = radius * subtendedAngle;
  double acceleration = newTargetLinearVelocity - targetLinearVelocity;
  if (abs(acceleration) >= MAX_ACCELERATION) {
    targetLinearVelocity += constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
    subtendedAngle = targetLinearVelocity / radius;
  }
  else
    targetLinearVelocity = newTargetLinearVelocity;
  */

  // /*
  //when tracking error for the purpose of PID tuning, it's important to update the target position to accurately track error
  if (errorCaptureEnabled) {
    double relativeThetaG = relativeTheta / 2.0d;
    double rG = rC * (2.0d * sin(relativeThetaG));
    targetVelocity->set(
      rG * sin(relativeThetaG),
      rG * cos(relativeThetaG),
      relativeTheta);
    targetPosition.set(targetVelocity);
    targetPosition.concat(&currentPosition);
  }
  // */

  /*
  SerialUSB.print("Target: ");
  SerialUSB.print(rC, 2);
  SerialUSB.print(" ");
  SerialUSB.println(relativeTheta, 2);
  */

  leftWheel.moveArc(rC, relativeTheta);
  rightWheel.moveArc(rC, relativeTheta);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

//just drive straight
void Zippy::executeLinearMove(double targetLinearVelocity)
{
  //follow behind the target point while actively moving
  // if (targetPositionUpdated)
    // targetLinearVelocity *= FOLLOW_FACTOR;

  /*
  double acceleration = newTargetLinearVelocity - targetLinearVelocity;
  if (abs(acceleration) >= MAX_ACCELERATION)
    targetLinearVelocity += constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
  else
    targetLinearVelocity = newTargetLinearVelocity;
  */

  leftWheel.moveStraight(targetLinearVelocity);
  rightWheel.moveStraight(targetLinearVelocity);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

void Zippy::executeTurn(double deltaOrientation)
{
  // if (abs(deltaOrientation) < MIN_ANGULAR_EPSILON)
    // deltaOrientation = (deltaOrientation < 0.0d) ? -MIN_ANGULAR_EPSILON : MIN_ANGULAR_EPSILON;
  leftWheel.turn(deltaOrientation);
  rightWheel.turn(deltaOrientation);
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}

//stopped when the signal from the lighthouse is lost
// /*
void Zippy::stop()
{
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
  currentMovementState = MovementState::Stopped;
}
// */

void Zippy::stopErrorCapture()
{
  errorCaptureEnabled = false;
}
