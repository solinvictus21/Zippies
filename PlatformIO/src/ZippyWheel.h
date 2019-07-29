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

  // void setInput(double velocityTurnRadius, double velocityOrientation);
  void setInputWithTurn(double linearVelocity, double angularVelocity);
  void setInputVelocity(double linearVelocity) { wheelInput = linearVelocity; }
  double getInput() const { return wheelInput; }

  void turn(double relativeTargetOrientation);
  void moveStraight(double linearVelocity);
  void moveWithTurn(double linearVelocity, double angularVelocity);
  void turnArc(double centerOffset, double angularVelocity) {
    wheelSetPoint = (centerOffset - wheelRadialOffset) * angularVelocity;
    wheelPID.Compute();
  }

  void stop() {
    wheelSetPoint = 0.0d;
    wheelPID.Compute();
  }

  double getSetPoint() const { return wheelSetPoint; }

  double getOutput() const { return wheelOutput; }

};

#endif
