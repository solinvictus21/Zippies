#include "ZippyWheel.h"

// #define WHEEL_Kp                                66.00d
// #define WHEEL_Ki                                 0.00d
// #define WHEEL_Kd                                 7.70d
#define WHEEL_Kp                               110.00d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                 9.50d
#define WHEEL_MAX_POWER                      60000.00d

// #define WHEEL_INTER_AXLE_POWER_LOSS              0.942886053042126d
#define WHEEL_INTER_AXLE_POWER_LOSS              1.110965890978642d

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

void ZippyWheel::setInputWithTurn(double linearVelocity, double angularVelocity)
{
  wheelInput = turnLength(linearVelocity, angularVelocity);
}

void ZippyWheel::turn(double angularVelocity)
{
  //calculate the new target velocity
  wheelSetPoint = -wheelRadialOffset * 2.0d * angularVelocity;
  // wheelSetPoint *= WHEEL_INTER_AXLE_POWER_LOSS;
  wheelPID.Compute();
}

void ZippyWheel::moveWithTurn(double linearVelocity, double angularVelocity)
{
  wheelSetPoint = turnLength(linearVelocity, angularVelocity);
  wheelPID.Compute();
}

void ZippyWheel::moveStraight(double linearVelocity)
{
  wheelSetPoint = linearVelocity;
  wheelPID.Compute();
}

double ZippyWheel::turnLength(double linearVelocity, double angularVelocity)
{
  double wheelTurnRadius = (linearVelocity / (2.0d * sin(angularVelocity))) - wheelRadialOffset;
  return wheelTurnRadius * 2.0d * angularVelocity;
  // return (centerTurnRadius + wheelRadialOffset) * 2.0d * deltaOrientation;
  /*
  double offsetRadius = centerTurnRadius + wheelRadialOffset;
  double desiredPower = offsetRadius * 2.0d * deltaOrientation;
  if (centerTurnRadius * wheelRadialOffset < 0.0d)
    return desiredPower;

  //in turns, the wheel on the outside of the turn is moving faster and so will experience power loss from the wheel
  //on the inside of the turn pulling or pushing on it along the axis of rotation
  // if (abs(centerTurnRadius) < abs(wheelRadialOffset))
    // return desiredPower * WHEEL_INTER_AXLE_POWER_LOSS;

  double dragPowerLoss = 1.0 + pow(cos(atan(offsetRadius / (2.0d * wheelOffset.getY()))), 2.0d);
  return desiredPower * dragPowerLoss;
  // */
}
