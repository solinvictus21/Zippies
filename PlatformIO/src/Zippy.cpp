
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "paths/ZPathPlanner.h"
#include "ZippyRoutine.h"

//the number of milliseconds between each time we evaluate the current position of the Zippy and adjust its motors
#define LOOP_INTERVAL_MS                         25

//the time to pause between the moment the lighthouse signal is detected after it is lost and the moment we start moving again
//adding an initial pause before moving allows time to completely set the Zippy down and the position detection to stabilize
#define INITIAL_PAUSE_TIME                     2000
#define INITIAL_LINEAR_VELOCITY                 200

#define WHEEL_OFFSET_X                           16.7d
#define WHEEL_OFFSET_Y                            5.9d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
// #define LINEAR_POSITION_EPSILON                            5.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
// #define ANGULAR_POSITION_EPSILON                  0.05235987755983d
// #define LINEAR_MAX_VELOCITY                     200.0d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#define LINEAR_MIN_POWER                       3500.00d

// #define TIMING_BEATS_MULTIPLIER        2

double getReflectedAngle(const KPosition* p);
// double getReflectedTurn(const KPosition* p);

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
        currentPath->interpolate(interpolatedTime, &targetPosition, &reverseMotion);
        break;
      }

      currentPath->interpolate(1.0d, &targetPosition, &reverseMotion);
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
    currentPath->interpolate(interpolatedTime, &targetPosition, &reverseMotion);
  }
}

void Zippy::planNextPath(unsigned long currentTime)
{
  if (currentPath != NULL) {
    //we need to capture the last point on this path segment to use as the starting
    //position to plan the move along the next path segment
    currentPath->interpolate(1.0d, &targetPosition, &reverseMotion);
    delete currentPath;
  }

  if (routineIndex >= ROUTINE_POSITION_COUNT) {
    currentPath = NULL;
    routineIndex = 0;
    currentState = WaitingForLighthouse;
    return;
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

  // driveArc(&relativeTargetPosition);
  driveHalfBiArc(&relativeTargetPosition);
}

void Zippy::driveHalfBiArc(const KPosition* relativeTargetPosition)
{
  double t2x = sin(relativeTargetPosition->orientation);
  double t2y = cos(relativeTargetPosition->orientation);

  //v dot v = (v.x * v.x) + (v.y * v.y)
  double vDotV = relativeTargetPosition->vector.getD2();

  //t = t1 + t2
  //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
  //v dot t = (v.x * t.x)                               + (v.y * t.y)
  //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
  //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
  double vDotT = (relativeTargetPosition->vector.getX() * t2x) +
    (relativeTargetPosition->vector.getY() * (1.0d + t2y));
  // print("vDotT is zero:", distanceZero(vDotT))

  //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
  //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
  //          = t2.y
  // double t1DotT2 = t2y;

  // print("Planned complex bi-arc.")
  //precalc = 2 * (1 - (t1 dot t2))
  double t1DotT2Inv2 = 2.0 * (1.0 - t2y);
  double discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) );

  //choose to follow the reverse path if it means less of a turn is required toward the final orientation
  double turnTowardTarget = 2.0d * relativeTargetPosition->vector.getOrientation();
  double turnTowardOrientation = subtractAngles(relativeTargetPosition->orientation, turnTowardTarget);
  bool reverseMotion = abs(turnTowardTarget + turnTowardOrientation) > M_PI;
  double d;
  if (reverseMotion) {
    //move backward
    d = ( -vDotT - discrim ) / t1DotT2Inv2;
  }
  else {
    //move forward
    d = ( -vDotT + discrim ) / t1DotT2Inv2;
  }

  //now find the "knot" (the connection point between the arcs)
  //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
  double pmx = ( relativeTargetPosition->vector.getX() + (d * (-t2x)) ) / 2.0;
  double pmy = ( relativeTargetPosition->vector.getY() + (d * (1.0d - t2y)) ) / 2.0;

  double linearVelocity = sqrt(pow(pmx, 2.0d) + pow(pmy, 2.0d));
  double angularVelocity = atan(pmx / pmy);
  if (pmy < 0.0d)
    linearVelocity = -linearVelocity;
  leftWheel.moveWithTurn(linearVelocity, angularVelocity);
  rightWheel.moveWithTurn(linearVelocity, angularVelocity);
}

void Zippy::driveArc(const KPosition* relativeTargetPosition)
{
  //first determine the single arc which would move us to be directly on the ray represented by the vector
  //to the target position and the target orientation we are trying to achieve from that position; since
  //the change in our current orientation will be twice the angle to the goal, that means that our goal
  //should be along the line represented by half of the change in orientation from our current orientation
  //toward our target orientation
  double arcAngle = relativeTargetPosition->orientation / 2.0;

  //now determine the intersection point from the direction of our goal arc to that ray
  //the distance along that arc angle is (tv X to) / (aa X to), where
  //  tv = vector to the current target position
  //  to = the target orientation
  //  aa = the angle of the ideal arc to the ray of the target orientation from the target position

  //in a more generalized use case, we would first calculate (aa X to) and check if this value
  //is zero before proceeding, but that can only happen when the current orientation is parallel
  //to the target orientation, and we've already checked for that condition above
  double sinA = sin(arcAngle);
  double cosA = cos(arcAngle);
  double sinO = sin(relativeTargetPosition->orientation);
  double cosO = cos(relativeTargetPosition->orientation);

  //the distance to the intersection point from vector A to ray bB is calculated by dividing the cross
  //product of b and B with the cross product of A and B
  double intersectionLength =
      ( (relativeTargetPosition->vector.getX() * cosO) - (relativeTargetPosition->vector.getY() * sinO) ) /
          ( (sinA * cosO) - (cosA * sinO) );

  //once we have the distance from A to the intersection point with bB, then the intersection point is...
  KVector2 orientationIntersection(
      intersectionLength * sinA,
      intersectionLength * cosA);

  double deltaDotDelta = orientationIntersection.dotVector(&orientationIntersection);
  double n2DotDelta = (-2.0 * orientationIntersection.getX());
  double arcCenterOffset = -deltaDotDelta / n2DotDelta;
  //now we have a radius, distance, and angle for an ideal arc from our current position and orientation
  //to a position along the line of target orientation which will arrive precisely aligned with that line

  //but our primary goal is to maintain alignment with the line from the target position that is orthogonal
  //to the orientation line; the goal is for the target position along the orientation line to "sweep" the
  //robot forward along the orientation line
  leftWheel.turnArc(arcCenterOffset, relativeTargetPosition->orientation);
  rightWheel.turnArc(arcCenterOffset, relativeTargetPosition->orientation);

  /*
  //first determine the single arc which would move us to be directly on the ray represented by the vector
  //to the target position and the target orientation we are trying to achieve from that position; since
  //the change in our current orientation will be twice the angle to the goal, that means that our goal
  //should be along the line represented by half of the change in orientation from our current orientation
  //toward our target orientation
  double arcAngle = relativeTargetPosition.orientation / 2.0d;

  //now determine the intersection point from the direction of our goal arc to that ray
  //the distance along that arc angle is (tv X to) / (aa X to), where
  //  tv = vector to the current target position
  //  to = the target orientation
  //  aa = the angle of the ideal arc to the ray of the target orientation from the target position

  //in a more generalized use case, we would first calculate (aa X to) and check if this value
  //is zero before proceeding, but that can only happen when the current orientation is parallel
  //to the target orientation, and we've already checked for that condition above
  double sinA = sin(arcAngle);
  double sinO = sin(relativeTargetPosition.orientation);
  double cosO = cos(relativeTargetPosition.orientation);
  double intersectionLength =
    ( (relativeTargetPosition.vector.getX() * cosO) + (relativeTargetPosition.vector.getY() * sinO) ) /
    ( (sinA * cosO) + (cos(arcAngle) * sinO) );
  double radius = intersectionLength * sinA;

  //now we have a radius, distance, and angle for an ideal arc from our current position and orientation
  //to a position along the line of target orientation which will arrive precisely aligned with that line

  //but rather than move the full arc, we will instead move along that arc to the point which will be
  //nearest the current target position; the end result will be that we'll always stay nearest the target
  //position that we can while also pointing as much toward the target orientation as we can

  //the point along the ideal arc that will be nearest the target position is along the ray from the
  //center of our ideal arc toward the current target position
  double angleOfBisection = atan((relativeTargetPosition.vector.getX() - radius) /
    relativeTargetPosition.vector.getY());

  //
  // double turnTowardTarget = atan(relativeTargetPosition.vector.getX() /
    // relativeTargetPosition.vector.getY());
  */

  /*
  //this is a variation of several possible "default" driving modes
  double linearVelocity = relativeTargetPosition.vector.dotOrientation(relativeTargetPosition.orientation);
  bool facingTowardTarget = relativeTargetPosition.vector.getY() > 0.0d;
  bool targetTowardOrientation = linearVelocity > 0.0d;
  double angularVelocity;
  if (facingTowardTarget) {
    if (!targetTowardOrientation) {
      //no matter whether we're reversed or not, we should be facing the orientation; in this case, that
      //means we need to turn fully away from the target
      // angularVelocity = addAngles(relativeTargetPosition.vector.getOrientation(), M_PI);
      angularVelocity = getReflectedAngle(&relativeTargetPosition);
      leftWheel.turn(angularVelocity);
      rightWheel.turn(angularVelocity);
      return;
    }
    else if (reverseMotion) {
      //facing the target, the target is toward the orientation, but we're in reverse, which means that
      //the target is heading for us
      angularVelocity = getReflectedAngle(&relativeTargetPosition);
      leftWheel.turn(angularVelocity);
      rightWheel.turn(angularVelocity);
      return;
    }
  }
  else {
    //not facing toward the target
    if (targetTowardOrientation) {
      //target should be behind us and the orientation in front of us, but the target is toward
      //the orientation, and we should be facing the orientation, so regardless of whether or not
      //we're in reverse, we should face the target
      angularVelocity = relativeTargetPosition.vector.arctan2();
      leftWheel.turn(angularVelocity);
      rightWheel.turn(angularVelocity);
      return;
    }
    else if (!reverseMotion) {
      //not facing the target, but the target is not heading toward the orientation, and we're not in
      //reverse, which means that the target is heading our way
      angularVelocity = getReflectedAngle(&relativeTargetPosition);
      leftWheel.turn(angularVelocity);
      rightWheel.turn(angularVelocity);
      return;
    }
  }

  angularVelocity = relativeTargetPosition.vector.arctan();
  if (linearVelocity == 0.0d) {
    leftWheel.turn(angularVelocity);
    rightWheel.turn(angularVelocity);
    return;
  }

  linearVelocity = constrain(linearVelocity, -LINEAR_MAX_VELOCITY, LINEAR_MAX_VELOCITY);
  leftWheel.moveWithTurn(linearVelocity, angularVelocity);
  rightWheel.moveWithTurn(linearVelocity, angularVelocity);
  */
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


double getReflectedAngle(const KPosition* p)
{
  double sinO = sin(p->orientation);
  double cosO = cos(p->orientation);
  double projectedLength = abs((p->vector.getX() * sinO) + (p->vector.getY() * cosO));
  return atan2(
    p->vector.getX() + (2.0d * projectedLength * sinO),
    p->vector.getY() + (2.0d * projectedLength * cosO));
}

/*
double getReflectedTurn(const KPosition* p)
{
  double sinO = sin(p->orientation);
  double cosO = cos(p->orientation);
  double projectedLength = abs((p->vector.getX() * sinO) + (p->vector.getY() * cosO));
  return atan(
    p->vector.getX() + (2.0d * projectedLength * sinO) /
    p->vector.getY() + (2.0d * projectedLength * cosO));
}
*/
