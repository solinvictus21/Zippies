
#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KPosition.h"

class ZippyWheel
{

private:
  KVector2 wheelOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

  double turnRadius(double centerTurnRadius, double deltaOrientation);

public:
  ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    unsigned long pidUpdateInterval);

  void start();

  void setInput(
    double velocityTurnRadius,
    double velocityOrientation);

  void setInput(double input) { this->wheelInput = input; }

  void move(
    double targetPositionTurnRadius,
    double targetPositionOrientation);

  void stop() {
    wheelSetPoint = 0.0d;
    wheelPID.Compute();
  }

  double getOutput() { return wheelOutput; }

};

#endif
