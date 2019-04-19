#include "ZippyWheel.h"

#define WHEEL_Kp                                65.00d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                10.00d
#define WHEEL_MAX_POWER                      60000.00d

ZippyWheel::ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    unsigned long pidUpdateInterval)
  : wheelOffset(wheelOffsetX, wheelOffsetY),
    wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, P_ON_E, DIRECT)
{
  // wheelRadialOffset = wheelOffset.getX();
  wheelRadialOffset = 2.0d * wheelOffset.getD();
  if (wheelOffset.getX() < 0.0d)
    wheelRadialOffset = -wheelRadialOffset;
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

void ZippyWheel::setInput(double velocityTurnRadius, double velocityOrientation)
{
  //calculate the velocity for this wheel
  wheelInput = turnLength(velocityTurnRadius, velocityOrientation);
}

void ZippyWheel::turn(double targetPositionOrientation)
{
  //calculate the new target velocity
  wheelSetPoint = wheelRadialOffset * 2.0d * -targetPositionOrientation;
  wheelPID.Compute();
}

void ZippyWheel::move(double targetPositionTurnRadius, double targetPositionOrientation)
{
  //calculate the new target velocity
  wheelSetPoint = turnLength(targetPositionTurnRadius, targetPositionOrientation);
  wheelPID.Compute();
}

void ZippyWheel::moveStraight(double linearVelocity)
{
  wheelSetPoint = linearVelocity;
  wheelPID.Compute();
}

double ZippyWheel::turnLength(double centerTurnRadius, double deltaOrientation)
{
  return (centerTurnRadius + wheelRadialOffset) * 2.0d * -deltaOrientation;
}
