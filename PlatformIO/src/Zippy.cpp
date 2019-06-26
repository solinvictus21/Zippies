
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "commands/PauseMove.h"
#include "commands/LinearVelocityMove.h"
#include "commands/LinearTurn.h"
#include "commands/PathMove.h"
#include "commands/SyncWithPreamble.h"
#include "paths/ZPathPlanner.h"
#include "ZippyRoutine.h"

//the number of milliseconds between each time we evaluate the current position of the Zippy and adjust its motors
#define LOOP_INTERVAL_MS                         25

//the time to pause between the moment the lighthouse signal is detected after it is lost and the moment we start moving again
//adding an initial pause before moving allows time to completely set the Zippy down and the position detection to stabilize
#define INITIAL_PAUSE_TIME                     2000
#define INITIAL_LINEAR_VELOCITY                 200

#define WHEEL_OFFSET_X   16.7d
#define WHEEL_OFFSET_Y    5.9d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_POSITION_EPSILON                            5.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
#define ANGULAR_POSITION_EPSILON                  0.05235987755983d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       3500.00d

// #define TIMING_BEATS_MULTIPLIER        2

Zippy::Zippy(
  double startingX,
  double startingY,
  double startingOrientation)
  : startPosition(startingX, startingY, startingOrientation),
    leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, LOOP_INTERVAL_MS),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y, LOOP_INTERVAL_MS)
{
#ifdef PLATFORM_TINYSCREEN
  face.displayFace();
#endif
}

//start the lighthouse and motors; we don't start the wheel PIDs until we have a solid lighthouse signal
void Zippy::start(unsigned long currentTime)
{
  lighthouse.start();
  motors.start();
  lastUpdateTime = currentTime;
}

void Zippy::loop(unsigned long currentTime)
{
  //process the Lighthouse diode hits on every loop, but don't both recalculating the position on the floor
  //based on those raw hits until we actually want to use them because it's a lot of math to waste if we're
  //not ready to use the data
  lighthouse.loop(currentTime);
  if (currentTime - lastUpdateTime < LOOP_INTERVAL_MS)
    return;
  lastUpdateTime += LOOP_INTERVAL_MS;

  // SerialUSB.println("Calculating position.");
  if (!lighthouse.recalculate(currentTime)) {
    //we are no longer able to determine our current position; stop moving and stop executing further commands
    if (currentState != WaitingForLighthouse) {
      // SerialUSB.println("Lighthouse is no longer available.");
      if (currentPath != NULL) {
        delete currentPath;
        currentPath = NULL;
      }
      motors.writeCommand(COMMAND_ALL_PWM, 30000, 30000, 30000, 30000);
      currentState = WaitingForLighthouse;
    }
    return;
  }

  switch (currentState) {
    case WaitingForLighthouse:
    {
      //we're obviously now done with this phase
      leftWheel.start();
      rightWheel.start();
      const KPosition* currentPosition = lighthouse.getPosition();
      if (positionsEquivalent(currentPosition, &startPosition)) {
        //we're already in position; go directly to syncing with the preamble
        targetPosition.set(currentPosition);
        lighthouse.clearPreambleFlag();
        currentState = SyncingWithPreamble;
        return;
      }

      //move into position
      currentPath = planPath(
        currentPosition->vector.getX(),
        currentPosition->vector.getY(),
        currentPosition->orientation,
        startPosition.vector.getX(),
        startPosition.vector.getY(),
        startPosition.orientation);
      currentPathStartTime = currentTime;
      currentPathDeltaTime = (currentPath->getLength() / INITIAL_LINEAR_VELOCITY) * 1000.0d;
      currentState = MovingToInitialPosition;
      return;
    }

    case MovingToInitialPosition:
    {
      unsigned long deltaTime = currentTime - currentPathStartTime;
      if (deltaTime < currentPathDeltaTime) {
        double interpolatedTime = ((double)deltaTime) / ((double)currentPathDeltaTime);
        currentPath->interpolate(interpolatedTime, &targetPosition);
        break;
      }

      currentPath->interpolate(1.0d, &targetPosition);
      delete currentPath;
      currentPath = NULL;
      lighthouse.clearPreambleFlag();
      currentState = SyncingWithPreamble;
      break;
    }

    case SyncingWithPreamble:
      if (!lighthouse.foundPreamble())
        break;

      routineIndex = 0;
      planNextPath(currentTime);
      currentState = Executing;
      break;

    case Executing:
      processCurrentPath(currentTime);
      break;
  }

  processInput();
  executeMove();
  driveMotors();
}

void Zippy::processCurrentPath(unsigned long currentTime)
{
  unsigned long deltaTime = currentTime - currentPathStartTime;
  if (deltaTime > currentPathDeltaTime) {
    deltaTime -= currentPathDeltaTime;
    //plan the next path segment
    planNextPath(currentTime - deltaTime);
  }

  if (currentPath != NULL) {
    double interpolatedTime = ((double)deltaTime) / ((double)currentPathDeltaTime);
    currentPath->interpolate(interpolatedTime, &targetPosition);
  }
}

void Zippy::planNextPath(unsigned long currentTime)
{
  if (currentPath != NULL) {
    //we need to capture the last point on this path segment to use as the starting
    //position to plan the move along the next path segment
    currentPath->interpolate(1.0d, &targetPosition);
    delete currentPath;
  }

  //plan the next path segment
  currentPath = planPath(
    targetPosition.vector.getX(),
    targetPosition.vector.getY(),
    targetPosition.orientation,
    ROUTINE[routineIndex].x,
    ROUTINE[routineIndex].y,
    ROUTINE[routineIndex].o);
  currentPathStartTime = currentTime;
  currentPathDeltaTime = ROUTINE[routineIndex].timing;
#ifdef TIMING_BEATS_MULTIPLIER
  currentPathDeltaTime *= TIMING_BEATS_MULTIPLIER;
#endif
  routineIndex++;
  if (routineIndex >= ROUTINE_POSITION_COUNT)
    routineIndex = 0;
}

void Zippy::processInput()
{
  //calculate our velocity relative to our previous position
  const KPosition* positionDelta = lighthouse.getPositionDelta();
  double linearVelocity = positionDelta->vector.getD();
  if (motors.inReverse())
    linearVelocity = -linearVelocity;

  double angularVelocity = positionDelta->orientation;
  if (sin(angularVelocity) == 0.0d) {
    //asymptote; direction is straight forward or straight backward, which would cause the
    //curve radius to be NaN / infinite
    leftWheel.setInputVelocity(linearVelocity);
    rightWheel.setInputVelocity(linearVelocity);
  }
  else {
    leftWheel.setInputWithTurn(linearVelocity, angularVelocity);
    rightWheel.setInputWithTurn(linearVelocity, angularVelocity);
  }
}

void Zippy::executeMove()
{
  const KPosition* position = lighthouse.getPosition();
  KPosition relativeTargetPosition(
      targetPosition.vector.getX() - position->vector.getX(),
      targetPosition.vector.getY() - position->vector.getY(),
      subtractAngles(targetPosition.orientation, position->orientation));
  relativeTargetPosition.vector.rotate(-position->orientation);

  if (distance2Zero(relativeTargetPosition.vector.getD2())) {
    //no linear movement needed
    if (angleZero(relativeTargetPosition.orientation)) {
      //no angular movement needed either; just stop
      leftWheel.stop();
      rightWheel.stop();
      return;
    }

    leftWheel.turn(relativeTargetPosition.orientation);
    rightWheel.turn(relativeTargetPosition.orientation);
    return;
  }

  if (angleZero(relativeTargetPosition.orientation)) {
    if (distanceZero(relativeTargetPosition.vector.getX())) {
      leftWheel.moveStraight(relativeTargetPosition.vector.getY());
      rightWheel.moveStraight(relativeTargetPosition.vector.getY());
      return;
    }
  }

  double angularVelocity = atan(relativeTargetPosition.vector.getX() / relativeTargetPosition.vector.getY());
  /*
  double deltaToTargetOrientation = subtractAngles(relativeTargetPosition.orientation, 2.0d * angularVelocity);
  if (abs(deltaToTargetOrientation) > M_PI_2) {
    leftWheel.turn(-angularVelocity);
    rightWheel.turn(-angularVelocity);
    return;
  }
  */

  //move directly toward the point
  double linearVelocity = relativeTargetPosition.vector.getD();
  if (relativeTargetPosition.vector.getY() < 0.0d)
    linearVelocity = -linearVelocity;

  leftWheel.moveWithTurn(linearVelocity, angularVelocity);
  rightWheel.moveWithTurn(linearVelocity, angularVelocity);
}

void Zippy::driveMotors()
{
  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
  /*
  if (abs(left) < 10.0d && abs(right) < 10.0d) {
    //stop when the requested power is below a certain threshold
    motors.writeCommand(COMMAND_ALL_PWM, 30000, 30000, 30000, 30000);
    return;
  }
  */

  left = saturate(left, LINEAR_MIN_POWER);
  right = saturate(right, LINEAR_MIN_POWER);
  motors.writeCommand(COMMAND_ALL_PWM,
      left > 0 ? left : 0,
      left < 0 ? -left : 0,
      right > 0 ? right : 0,
      right < 0 ? -right : 0);
}

/*
double Zippy::centerTurnRadius(double distanceDelta, double orientationDelta)
{
  if (orientationDelta == 0.0d)
    return 0.0d;
  return distanceDelta / (2.0d * sin(-orientationDelta));
}
// */

double Zippy::saturate(double a, double b)
{
  if (a == 0.0d)
    return 0.0d;

  return (a < 0.0d ? -b : b) + a;
}
