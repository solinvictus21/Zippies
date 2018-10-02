
#include <SPI.h>
#include "Zippy.h"
#include "ZippyConfig.h"
#include "PathData.h"

#define WHEEL_OFFSET_X   16.5d
#define WHEEL_OFFSET_Y    5.9d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_EPSILON                           20.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (5 degrees)
#define ANGULAR_EPSILON                           0.087266462599716d

//TUNING - PID CONFIGURATION
//the maximum percentage we allow the linear velocity input to change from the previous input during acceleration forward or backward
#define LINEAR_MAX_INPUT_CHANGE_FACTOR            0.20d
//the minimum amount of absolute change in linear velocity input to allow when the percentage of the previous value is below this threshold
//the application of both of these values caps acceleration changes to prevent PID overshoot
#define LINEAR_MIN_INPUT_CHANGE                  40.00d

#define LINEAR_Kp                                 5.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.00d
// #define LINEAR_Kp                                 0.90d
// #define LINEAR_Ki                                 0.00d
// #define LINEAR_Kd                                 0.05d
#define LINEAR_MAX_POWER                      50000.00d

#define ANGULAR_Kp                             2000.00d
#define ANGULAR_Ki                                0.00d
#define ANGULAR_Kd                                0.00d
#define ANGULAR_MAX_POWER                     10000.00d
#define ANGULAR_SPEED_COMPENSATION                0.40d

//TUNING - PCM OUTPUT
//the threshold below which the wheels are set to stop entirely
#define MOTOR_MIN_THRESHOLD                       0.00d
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       4400.00d
#define ANGULAR_MIN_POWER                      5200.00d

double snap(double motorPower, double zeroThreshold, double minimumMagnitude);
double pad(double motorPower, double minimumMagnitude);

Zippy::Zippy(unsigned long pui)
  : pidUpdateInterval(pui),
#ifdef MOTOR_MODEL_COMBINED
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT),
    angularPID(&angularInput, &angularOutput, &angularSetPoint, ANGULAR_Kp, ANGULAR_Ki, ANGULAR_Kd, P_ON_E, DIRECT)
{
  linearPID.SetSampleTime(pidUpdateInterval);
  linearPID.SetOutputLimits(-LINEAR_MAX_POWER, LINEAR_MAX_POWER);
  angularPID.SetSampleTime(pidUpdateInterval);
  angularPID.SetOutputLimits(-ANGULAR_MAX_POWER, ANGULAR_MAX_POWER);
#else
    leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, pidUpdateInterval),
    rightWheel(WHEEL_OFFSET_X,  WHEEL_OFFSET_Y, pidUpdateInterval)
{
#endif
  face.displayFace();
}

void Zippy::move(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = false;
  stopped = false;
}

void Zippy::turn(double orientation)
{
  currentTargetPosition.orientation = orientation;
  orientationUpdated = true;
  prioritizeOrientation = true;
  stopped = false;
}

void Zippy::turnAndMove(double x, double y, double orientation)
{
  currentTargetPosition.vector.set(x, y);
  currentTargetPosition.orientation = orientation;
  positionUpdated = true;
  orientationUpdated = true;
  prioritizeOrientation = true;
  stopped = false;
}

//start all of our peripherals
void Zippy::start()
{
#ifdef PLATFORM_TINYSCREEN
  motors.start();
#endif

#ifdef MOTOR_MODEL_COMBINED
  linearPID.SetMode(MANUAL);
  angularPID.SetMode(MANUAL);

  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  linearSetPoint = 0.0d;
  linearInput = 0.0d;
  linearOutput = 0.0d;
  angularSetPoint = 0.0d;
  angularInput = 0.0d;
  angularOutput = 0.0d;

  linearPID.SetMode(AUTOMATIC);
  angularPID.SetMode(AUTOMATIC);
#else
  leftWheel.start();
  rightWheel.start();
#endif
}

bool Zippy::loop(const KPosition* currentPosition,
                 const KPosition* currentPositionDelta)
{
  /*
  setMotors(-LINEAR_MIN_POWER, -LINEAR_MIN_POWER);
  // setMotors(-ANGULAR_MIN_POWER, ANGULAR_MIN_POWER);
  // setMotors(7200.0d, 0.0d);
  return false;
  // */
  // /*
#ifdef MOTOR_MODEL_COMBINED
  if (stopped) {
    //need to continue to let the PIDs loop or they get out of sync with the control loop
    linearInput = 0.0d;
    linearSetPoint = 0.0d;
    angularInput = 0.0d;
    linearPID.Compute();
    angularPID.Compute();
    return true;
  }

  //calculate the vector from our current position to the current target position from the perspective of the current orientation
  KVector2 deltaPosition(currentTargetPosition.vector.getX() - currentPosition->vector.getX(),
      currentTargetPosition.vector.getY() - currentPosition->vector.getY());
  deltaPosition.rotate(-currentPosition->orientation);

  bool atTarget = calculateLinearInput(&deltaPosition, currentPosition, currentVelocity);
  atTarget &= calculateAngularInput(&deltaPosition, currentPosition, currentVelocity);

  linearPID.Compute();
  angularPID.Compute();

  double left = linearOutput+angularOutput;
  double right = linearOutput-angularOutput;
  double pad = left * right < 0.0d ? ANGULAR_MIN_POWER : LINEAR_MIN_POWER;
  setMotors(snap(left, MOTOR_MIN_THRESHOLD, pad),
      snap(right, MOTOR_MIN_THRESHOLD, pad));

  return atTarget;
#else
  bool atTarget = leftWheel.loop(currentPosition, currentPositionDelta, &currentTargetPosition);
  atTarget &= rightWheel.loop(currentPosition, currentPositionDelta, &currentTargetPosition);

  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
  double padding = left * right < 0.0d ? ANGULAR_MIN_POWER : LINEAR_MIN_POWER;
  // setMotors(snap(-left, MOTOR_MIN_THRESHOLD, padding),
      // snap(-right, MOTOR_MIN_THRESHOLD, padding));
  setMotors(pad(-left, LINEAR_MIN_POWER),
      pad(-right, LINEAR_MIN_POWER));

  return atTarget;
#endif
  // */
}

#ifdef MOTOR_MODEL_COMBINED
//calculate the linear velocity input; since the ZIppy can only move along its Y axis, we only consider the Y offset
bool Zippy::calculateLinearInput(const KVector2* deltaPosition,
                                 const KPosition* currentPosition,
                                 const KPosition* currentVelocity)
{
  // linearInput = currentVelocity->vector.getD();
  linearInput = currentVelocity->vector.getD() * ((double)pidUpdateInterval) / 1000.0d;
  if (abs(subtractAngles(currentVelocity->vector.getOrientation(), currentPosition->orientation)) > M_PI_2)
    linearInput = -linearInput;

  if (!positionUpdated && deltaPosition->getD() < LINEAR_EPSILON) {
    linearSetPoint = 0.0d;
    return abs(linearInput) < LINEAR_EPSILON;
  }
  positionUpdated = false;

  double deltaY = deltaPosition->getY();
  double inputDelta = - deltaY - linearSetPoint;
  // double inputDelta = - ((deltaPosition * 1000.0d) / ((double)pidUpdateInterval)) - linearSetPoint;

  //limit the amount of acceleration to clamp down on PID overshoot; deceleration of any amount is fine
  bool sameDirection = deltaY * linearSetPoint >= 0.0d;
  if (!sameDirection ||
      (sameDirection && abs(deltaY) > abs(linearSetPoint)))
  {
    double maxInputChange = max(abs(linearSetPoint) * LINEAR_MAX_INPUT_CHANGE_FACTOR, LINEAR_MIN_INPUT_CHANGE);
    inputDelta = constrain(inputDelta, -maxInputChange, maxInputChange);
  }
  linearSetPoint += inputDelta;

  return false;
}

//calculate the rotational velocity input
bool Zippy::calculateAngularInput(const KVector2* deltaPosition,
                                  const KPosition* currentPosition,
                                  const KPosition* currentVelocity)
{
  double deltaOrientation;
  if (prioritizeOrientation ||
      inReverse == (deltaPosition->getY() > 0.0d))
    deltaOrientation = subtractAngles(currentTargetPosition.orientation, currentPosition->orientation);
  else
    deltaOrientation = deltaPosition->getOrientation();

  if (inReverse)
    deltaOrientation = addAngles(deltaOrientation, M_PI);

  if (!orientationUpdated && abs(deltaOrientation) < ANGULAR_EPSILON) {
    angularInput = 0.0d;
    return true;
  }
  orientationUpdated = false;

  angularInput = constrain(deltaOrientation, -0.5d, 0.5d);
  // double absVelocity = currentVelocity->vector.getD();
  // if (absVelocity >= LINEAR_EPSILON)
    // angularInput = pad(angularInput, ANGULAR_SPEED_COMPENSATION * (absVelocity - LINEAR_EPSILON) / 1000.0d);

  return false;
}
#endif

void Zippy::stop()
{
  setMotors(0.0d, 0.0d);
  stopped = true;
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft < 0 ? -motorLeft: 0,
      motorLeft > 0 ? motorLeft: 0,
      motorRight < 0 ? -motorRight: 0,
      motorRight > 0 ? motorRight: 0);
}

double snap(double motorPower, double zeroThreshold, double minimumMagnitude)
{
  if (abs(motorPower) < zeroThreshold)
    return 0.0d;

  if (motorPower > 0.0d)
    return minimumMagnitude + (motorPower - zeroThreshold);
  else if (motorPower < 0.0d)
    return -minimumMagnitude + (motorPower + zeroThreshold);

  return 0.0d;
}

double pad(double motorPower, double minimumMagnitude)
{
  if (motorPower > 0.0d)
    return minimumMagnitude + motorPower;
  else if (motorPower < 0.0d)
    return -minimumMagnitude + motorPower;

  return 0.0d;
}
