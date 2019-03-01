#include "ZippyWheel.h"

#define WHEEL_Kp                               130.00d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                 7.60d
#define WHEEL_MAX_POWER                      60000.00d

double curveLength(const KVector2* v);
double curveLength(double relativeDistance, double relativeOrientation);
double relativeDistance(const KVector2* v);

ZippyWheel::ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    unsigned long pidUpdateInterval)
  : wheelOffset(wheelOffsetX, wheelOffsetY),
    wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, P_ON_E, DIRECT)
{
  wheelPID.SetSampleTime(pidUpdateInterval);
  wheelPID.SetOutputLimits(-WHEEL_MAX_POWER, WHEEL_MAX_POWER);
}

void ZippyWheel::start()
{
  wheelPID.SetMode(MANUAL);
  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  wheelSetPoint = 0.0d;
  wheelInput = 0.0d;
  wheelOutput = 0.0d;
  wheelPID.SetMode(AUTOMATIC);
}

void ZippyWheel::setInput(const KVector2* relativeVelocity, double relativeVelocityOrientation)
{
  KVector2 wheelPositionDelta(&wheelOffset);
  wheelPositionDelta.rotate(relativeVelocityOrientation);
  wheelPositionDelta.addVector(relativeVelocity);
  wheelPositionDelta.subtractVector(&wheelOffset);
  // wheelPositionDelta.rotate(-relativeVelocity->getOrientation());
  wheelInput = curveLength(&wheelPositionDelta);
}

void ZippyWheel::turn(double relativeTargetOrientation)
{
  KVector2 wheelPositionDelta(&wheelOffset);
  wheelPositionDelta.rotate(relativeTargetOrientation);
  wheelPositionDelta.subtractVector(&wheelOffset);
  // wheelPositionDelta.rotate(-relativeTargetOrientation);
  wheelSetPoint = curveLength(&wheelPositionDelta);
  wheelPID.Compute();
}

void ZippyWheel::move(double linearVelocity, double angularVelocity)
{
  KVector2 wheelPositionDelta(&wheelOffset);
  wheelPositionDelta.rotate(angularVelocity);
  wheelPositionDelta.setY(wheelPositionDelta.getY() + linearVelocity);
  wheelPositionDelta.rotate(angularVelocity);
  wheelPositionDelta.subtractVector(&wheelOffset);
  // wheelPositionDelta.rotate(-angularVelocity);
  wheelSetPoint = curveLength(&wheelPositionDelta);
  wheelPID.Compute();
}

void ZippyWheel::setInput2(double velocityTurnRadius, double velocityOrientation)
{
  //calculate the velocity for this wheel
  wheelInput = turnLength(velocityTurnRadius, velocityOrientation);
}

void ZippyWheel::turn2(double targetPositionOrientation)
{
  //calculate the new target velocity
  wheelSetPoint = wheelOffset.getX() * -targetPositionOrientation;
  wheelSetPoint = relativeDistance(&wheelOffset) * -targetPositionOrientation;
  wheelPID.Compute();
}

void ZippyWheel::move2(double targetPositionTurnRadius, double targetPositionOrientation)
{
  //calculate the new target velocity
  wheelSetPoint = turnLength(targetPositionTurnRadius, targetPositionOrientation);
  wheelPID.Compute();
}

double ZippyWheel::turnLength(double centerTurnRadius, double deltaOrientation)
{
  // return (centerTurnRadius + wheelOffset.getX()) * 2.0d * -deltaOrientation;
  return (centerTurnRadius + relativeDistance(&wheelOffset)) * 2.0d * -deltaOrientation;
}
