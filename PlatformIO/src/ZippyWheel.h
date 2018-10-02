
#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KPosition.h"

#define WHEEL_Kp                                 5.80d
#define WHEEL_Ki                                 0.00d
#define WHEEL_Kd                                 0.00d
#define WHEEL_MAX_POWER                      65000.00d

class ZippyWheel
{

private:
  KVector2 wheelOffset;

  double wheelSetPoint = 0.0d;
  double wheelInput = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

public:
  ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    unsigned long pidUpdateInterval)
  : wheelOffset(wheelOffsetX, wheelOffsetY),
    wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, WHEEL_Kp, WHEEL_Ki, WHEEL_Kd, P_ON_E, DIRECT)
  {
    wheelPID.SetSampleTime(pidUpdateInterval);
    wheelPID.SetOutputLimits(-WHEEL_MAX_POWER, WHEEL_MAX_POWER);
  }

  void start();

  bool loop(
    const KPosition* currentPosition,
    const KPosition* currentPositionDelta,
    const KPosition* targetPosition);

  double getOutput() { return wheelOutput; }

};

#endif
