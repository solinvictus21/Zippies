
#include <SPI.h>
#include "Zippy.h"
#include "PathData.h"
#include "ZippyRoutine.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"

//wheel PID config
#define WHEEL_Kp                               100.00d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                20.00d
#define WHEEL_MAX_POWER                      60000.00d

#define PID_INTERVAL                           17

#define DISPLAY_UPDATE_INTERVAL              5000

unsigned long lastDisplayUpdateTime;
Zippy::Zippy()
  : leftWheel(-WHEEL_OFFSET_X, -WHEEL_OFFSET_Y,
        WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, WHEEL_MAX_POWER, PID_INTERVAL),
    rightWheel(WHEEL_OFFSET_X, WHEEL_OFFSET_Y,
        WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, WHEEL_MAX_POWER, PID_INTERVAL)
{
#ifdef PLATFORM_TINYSCREEN
  face.displayFace();
#endif
}

//start the lighthouse and motors; we don't start the wheel PIDs until we have a solid lighthouse signal
void Zippy::start()
{
  motors.start();
  leftWheel.start();
  rightWheel.start();
  lastDisplayUpdateTime = micros() / 1000;
}

void Zippy::loop(unsigned long currentTime)
{
  if (currentTime - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime += DISPLAY_UPDATE_INTERVAL;
    // face.begin();
    face.clearScreen();
    face.displayLabelAndData(
        0,
        SCREEN_HEIGHT_PIXELS_2,
        "RE", rightWheel.getAverageError());
    face.displayLabelAndData(
        SCREEN_WIDTH_PIXELS_2,
        SCREEN_HEIGHT_PIXELS_2,
        "LE", leftWheel.getAverageError());
    face.displayLabelAndData(
        0,
        SCREEN_HEIGHT_PIXELS_2 + face.getFontHeight(),
        "R", (int)rightWheel.getMaxOutput());
    face.displayLabelAndData(
        SCREEN_WIDTH_PIXELS_2,
        SCREEN_HEIGHT_PIXELS_2 + face.getFontHeight(),
        "L", (int)leftWheel.getMaxOutput());
    leftWheel.clearError();
    rightWheel.clearError();
    // face.end();
  }
}

void Zippy::processInput(const KMatrix2* positionDelta)
{
  //process our PID inputs
  if (abs(positionDelta->position.atan()) < DOUBLE_EPSILON) {
    //asymptote; direction is straight forward or straight backward, which would cause the
    //curve radius to be NaN / infinite or, at the very least, create a radius that would
    //induce far too much Abbe error into the arc calculation
    double distance = positionDelta->position.getY();
    leftWheel.setInputStraight(distance);
    rightWheel.setInputStraight(distance);
  }
  else {
    double subtendedAngle = positionDelta->orientation.get();
    // double radius = positionDelta->position.getD() / (2.0 * sin(positionDelta->position.atan2()));
    // double radius = positionDelta->position.getD() / positionDelta->orientation.sin();
    // double radius = (M_PI - subtendedAngle) / 2.0d;
    // double radius = (motors.inReverse() ? -positionDelta->position.getD() : positionDelta->position.getD()) / (2.0d * sin(subtendedAngle / 2.0d));
    double radius = positionDelta->position.getY() / positionDelta->orientation.sin();
    leftWheel.setInputArc(radius, subtendedAngle);
    rightWheel.setInputArc(radius, subtendedAngle);
  }
}

void Zippy::executeMove(const KMatrix2* positionDelta, KMatrix2* relativeTarget)
{
  if (relativeTarget->position.getD2() < DOUBLE_EPSILON) {
    //this is actually just a turn
    executeTurn(positionDelta, relativeTarget->orientation.get());
    return;
  }

  processInput(positionDelta);
  //determine if we need to plan a bi-arc move first
  if (requiresBiArcMove(relativeTarget))
    calculateRelativeBiArcKnot(relativeTarget);

  if (abs(relativeTarget->position.atan()) < DOUBLE_EPSILON) {
    //just drive straight
    double distance = relativeTarget->position.getY();
    leftWheel.moveStraight(distance);
    rightWheel.moveStraight(distance);
  }
  else {
    //simple arc
    double subtendedAngle = 2.0d * relativeTarget->position.atan();
    double radius = relativeTarget->position.getD() / (2.0 * sin(relativeTarget->position.atan2()));
    leftWheel.moveArc(radius, subtendedAngle);
    rightWheel.moveArc(radius, subtendedAngle);
  }
  driveMotors();
}

void Zippy::executeTurn(const KMatrix2* positionDelta, double deltaOrientation)
{
  if (abs(deltaOrientation) < DOUBLE_EPSILON) {
    executeStop();
    return;
  }

  processInput(positionDelta);
  leftWheel.turn(deltaOrientation);
  rightWheel.turn(deltaOrientation);
  driveMotors();
}

void Zippy::executeStop()
{
  leftWheel.setInputStraight(0.0d);
  rightWheel.setInputStraight(0.0d);
  leftWheel.stop();
  rightWheel.stop();
  motors.stopMotors();
}

void Zippy::driveMotors()
{
  motors.setMotors(leftWheel.getOutput(), rightWheel.getOutput());
}
