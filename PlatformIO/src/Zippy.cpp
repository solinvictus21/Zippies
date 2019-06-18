
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "commands/PauseMove.h"
#include "commands/LinearVelocityMove.h"
#include "commands/LinearTurn.h"
#include "commands/PathMove.h"
#include "commands/SyncWithPreamble.h"

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
#define LINEAR_MIN_POWER                       3200.00d

Zippy::Zippy(
  double startingX,
  double startingY,
  double startingOrientation,
  PathMove* mp)
  : movementPath(mp),
    leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y, LOOP_INTERVAL_MS),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y, LOOP_INTERVAL_MS)
{
  //add a set of commands to move the Zippy into place and sync with the preamble
  ZippyMove** moves = new ZippyMove*[5];
  moves[0] = new PauseMove(INITIAL_PAUSE_TIME);
  moves[1] = new LinearVelocityMove(startingX, startingY, INITIAL_LINEAR_VELOCITY);
  moves[2] = new LinearTurn(startingOrientation, INITIAL_PAUSE_TIME, false);
  moves[3] = new SyncWithPreamble();
  moves[4] = mp;
  movementPath = new PathMove(moves, 5);

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
    if (lighthouseReady) {
      // SerialUSB.println("Lost lighthouse signal.");
      movementPath->end();
      stopMoving();
      lighthouseReady = false;
    }
    return;
  }
  else if (!lighthouseReady) {
    targetPosition.set(lighthouse.getPosition());
    movementPath->start(currentTime, this);
    lighthouseReady = true;
    return;
  }

  double timeRemaining = movementPath->loop(currentTime, this);
  if (timeRemaining) {
    movementPath->end();
    targetPosition.set(lighthouse.getPosition());
    movementPath->start(currentTime - timeRemaining, this);
  }
}

void Zippy::startMoving()
{
  if (isMoving)
    return;

  leftWheel.start();
  rightWheel.start();
  isMoving = true;
}

void Zippy::move(double x, double y, double orientation)
{
  targetPosition.set(x, y, orientation);
  if (!isMoving)
    return;

  processInput();

  const KPosition* position = lighthouse.getPosition();
  KPosition relativeTargetPosition(
      x - position->vector.getX(),
      y - position->vector.getY(),
      subtractAngles(orientation, position->orientation));
  relativeTargetPosition.vector.rotate(-position->orientation);

  // plotBiArc(&relativeTargetPosition);
  moveArc(&relativeTargetPosition);

  driveMotors();
}

void Zippy::plotBiArc(KPosition* relativeTargetPosition)
{
  //derived from formulae and helpful explanations provided from the following site
  //  http://www.ryanjuckett.com/programming/biarc-interpolation/
  double x = relativeTargetPosition->vector.getX();
  double y = relativeTargetPosition->vector.getY();
  double cosO = cos(relativeTargetPosition->orientation);
  double sinO = sin(relativeTargetPosition->orientation);

  double d;
  if (abs(relativeTargetPosition->orientation) == 0.0d) {
    if (y == 0.0d) {
      relativeTargetPosition->vector.multiply(0.5d);
      return;
    }

    //in this situation, our denominator becomes zero and d goes to infinity; handle this with different formulae
    double vDotT2 = (x * sinO) + (y * cosO);
    d = relativeTargetPosition->vector.getD2() / (4.0d * vDotT2);
  }
  else {
    //t = t1 + t2
    //  where t1 = current relative orientation = (0, 1)
    //        t2 = target relative orientation = (sin(o), cos(o))
    //v dot t = (v.x * t.x)           + (v.y * t.y)
    //        = (v.x * (t1.x + t2.x)) + (v.y * (t1.y + t2.y))
    //        = (v.x * (0 + sin(o)))  + (v.y * (1 + cos(o)))
    //        = (v.x * sin(o))        + (v.y * (1 + cos(o)))
    double vDotT = (x * sinO) + (y * (1.0d + cosO));
    //t1 dot t2 = (t1.x * t2.x) + (t1.y * t2.y)
    //          = (0 * sin(o)) + (1 * cos(o))
    //          = cos(o)
    //2 * (1 - (t1 dot t2)) = 2 * (1 - cos(o))
    double t1DotT2Inv2 = 2.0d * (1.0d - cosO);
    d = ( -vDotT + sqrt( pow(vDotT, 2.0d) + ( t1DotT2Inv2 * relativeTargetPosition->vector.getD2() ) ) )
        / t1DotT2Inv2;
  }

  //pm = (p2 + (d * (t1 - t2))) / 2
  relativeTargetPosition->vector.set(
    ((d * -sinO) + x) / 2.0d,
    ((d * (1.0d - cosO)) + y) / 2.0d);
}

void Zippy::moveArc(const KPosition* relativeTargetPosition)
{
  if (relativeTargetPosition->vector.getX() == 0.0d) {
    if (relativeTargetPosition->vector.getY() == 0.0d) {
      leftWheel.stop();
      rightWheel.stop();
    }
    else {
      double linearVelocity = relativeTargetPosition->vector.getY();
      leftWheel.moveStraight(linearVelocity);
      rightWheel.moveStraight(linearVelocity);
    }
    return;
  }

  double linearVelocity, angularVelocity;
  if (relativeTargetPosition->vector.getY() == 0.0d) {
    //handle asymptote at y = 0, since x/y would be NaN
    linearVelocity = abs(relativeTargetPosition->vector.getX());
    angularVelocity = relativeTargetPosition->vector.getX() < 0.0d ? -M_PI_2 : M_PI_2;
  }
  else {
    linearVelocity = relativeTargetPosition->vector.getD();
    if (relativeTargetPosition->vector.getY() < 0.0d)
      linearVelocity = -linearVelocity;
    angularVelocity = atan(relativeTargetPosition->vector.getX() / relativeTargetPosition->vector.getY());

    // linearVelocity = min(linearVelocity, 100.0d);
    /*
    double finalOrientation = subtractAngles(2.0d * angularVelocity, relativeTargetPosition->orientation);
    if (abs(finalOrientation) > M_PI_2) {
      angularVelocity = -angularVelocity;
      // angularVelocity = addAngles(angularVelocity,
        // subtractAngles(relativeTargetPosition->orientation, angularVelocity) / 2.0d);
    }
    // */
  }
  leftWheel.moveWithTurn(linearVelocity, angularVelocity);
  rightWheel.moveWithTurn(linearVelocity, angularVelocity);
}

void Zippy::moveBiArc(KPosition* relativeTargetPosition)
{
  if (relativeTargetPosition->vector.getY() < 0.0d)
    reversePlotBiArc(relativeTargetPosition);
  else
    plotBiArc(relativeTargetPosition);
  moveArc(relativeTargetPosition);
}

void Zippy::turn(double orientation)
{
  targetPosition.orientation = orientation;
  if (!isMoving)
    return;

  processInput();
  const KPosition* position = lighthouse.getPosition();
  double relativeOrientation = subtractAngles(orientation, position->orientation);
  leftWheel.turn(relativeOrientation);
  rightWheel.turn(relativeOrientation);
  driveMotors();
}

void Zippy::stopMoving()
{
  if (!isMoving)
    return;

  leftWheel.stop();
  rightWheel.stop();
  motors.writeCommand(COMMAND_ALL_PWM, 30000, 30000, 30000, 30000);
  isMoving = false;
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

void Zippy::driveMotors()
{
  double left = leftWheel.getOutput();
  double right = rightWheel.getOutput();
  left = saturate(left, LINEAR_MIN_POWER);
  right = saturate(right, LINEAR_MIN_POWER);
  motors.writeCommand(COMMAND_ALL_PWM,
      left > 0 ? left : 0,
      left < 0 ? -left : 0,
      right > 0 ? right : 0,
      right < 0 ? -right : 0);
}

void Zippy::reversePlotBiArc(KPosition* relativeTargetPosition)
{
  relativeTargetPosition->vector.set(
      -relativeTargetPosition->vector.getX(),
      -relativeTargetPosition->vector.getY());
  // relativeDirectionOfMotion = addAngles(relativeDirectionOfMotion, M_PI);
  plotBiArc(relativeTargetPosition);
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
