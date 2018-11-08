
#include "ZippyWheel.h"

// #define WHEEL_Kp                                22.00d
#define WHEEL_Kp                                50.00d
#define WHEEL_Ki                                 0.00d
// #define WHEEL_Kd                                 0.15d
#define WHEEL_Kd                                 2.00d
// #define WHEEL_Kd                                 0.40d
#define WHEEL_MAX_POWER                      60000.00d

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

void ZippyWheel::setInput(
  double velocityTurnRadius,
  double velocityOrientation)
{
  //calculate the velocity for this wheel
  wheelInput = turnRadius(velocityTurnRadius, velocityOrientation);
}

void ZippyWheel::move(
  double targetPositionTurnRadius,
  double targetPositionOrientation)
{
  //calculate the new target velocity
  wheelSetPoint = turnRadius(targetPositionTurnRadius, targetPositionOrientation);
  wheelPID.Compute();
}

double ZippyWheel::turnRadius(double centerTurnRadius, double deltaOrientation)
{
  return (centerTurnRadius + wheelOffset.getX()) * 2.0d * -deltaOrientation;
}
