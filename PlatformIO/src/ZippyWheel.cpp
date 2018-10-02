
#include "ZippyWheel.h"

#define POSITION_EPSILON                           20.0d
#define POSITIONDELTA_EPSILON                      10.0d

void ZippyWheel::start()
{
  wheelPID.SetMode(MANUAL);
  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  wheelSetPoint = 0.0d;
  wheelInput = 0.0d;
  wheelOutput = 0.0d;
  wheelPID.SetMode(AUTOMATIC);
}

bool ZippyWheel::loop(
  const KPosition* currentPosition,
  const KPosition* currentPositionDelta,
  const KPosition* targetPosition)
{
  KVector2 relativePositionDelta(&currentPositionDelta->vector);
  relativePositionDelta.rotate(-currentPosition->orientation);

  KVector2 wheelPositionDelta(&wheelOffset);
  wheelPositionDelta.rotate(currentPositionDelta->orientation);
  wheelPositionDelta.addVector(&relativePositionDelta);
  wheelPositionDelta.subtractVector(&wheelOffset);
  wheelInput = wheelPositionDelta.getY() > 0.0d ? wheelPositionDelta.getD() : -wheelPositionDelta.getD();
  // wheelInput = wheelPositionDelta.getY() +
      // (wheelOffset.getX() < 0.0d ? wheelPositionDelta.getX() : -wheelPositionDelta.getX());

  KVector2 currentWheelPosition(&wheelOffset);
  currentWheelPosition.rotate(currentPosition->orientation);
  currentWheelPosition.addVector(&currentPosition->vector);

  KVector2 deltaWheelPosition(&wheelOffset);
  deltaWheelPosition.rotate(targetPosition->orientation);
  deltaWheelPosition.addVector(&targetPosition->vector);
  deltaWheelPosition.subtractVector(&currentWheelPosition);
  deltaWheelPosition.rotate(-currentPosition->orientation);

  // wheelSetPoint = deltaWheelPosition.getY() > 0.0d ? deltaWheelPosition.getD() : -deltaWheelPosition.getD();
  wheelSetPoint = deltaWheelPosition.getY() +
      (wheelOffset.getX() < 0.0d ? deltaWheelPosition.getX() : -deltaWheelPosition.getX());

  wheelPID.Compute();

  return deltaWheelPosition.getD() < POSITION_EPSILON && wheelPositionDelta.getD() < POSITIONDELTA_EPSILON;
}
