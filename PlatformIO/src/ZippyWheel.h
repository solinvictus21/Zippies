#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KPosition.h"

class ZippyWheel
{

private:
  KVector2 wheelOffset;
  double wheelRadialOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

  double turnLength(double centerTurnRadius, double deltaOrientation);
  // double power(double centerTurnRadius, double deltaOrientation);
  // double relativeDistance(const KVector2* v);

public:
  ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    unsigned long pidUpdateInterval);

  void start();

  void stop() {
    wheelSetPoint = wheelInput;
    wheelPID.Compute();
  }

  double getInput() const { return wheelInput; }
  double getSetPoint() const { return wheelSetPoint; }
  double getOutput() const { return wheelOutput; }

  void setInput(double velocityTurnRadius, double velocityOrientation);
  void turn(double relativeTargetOrientation);
  void move(double targetPositionTurnRadius, double targetPositionOrientation);
  void moveStraight(double linearVelocity);

};

#endif
