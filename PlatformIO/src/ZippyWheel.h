#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KMatrix2.h"

class ZippyWheel
{

private:
  double wheelRadialOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

public:
  ZippyWheel(
    double wro,
    double Kp, double Ki, double Kd, double outputLimit,
    unsigned long pidUpdateInterval)
    : wheelRadialOffset(wro),
      wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, Kp, Ki, Kd, P_ON_E, DIRECT)
  {
    wheelPID.SetSampleTime(pidUpdateInterval);
    wheelPID.SetOutputLimits(-outputLimit, outputLimit);
  }

  void setTunings(double p, double i, double d) {
    wheelPID.SetTunings(p, i, d);
  }

  void start() {
    wheelSetPoint = 0.0d;
    wheelOutput = 0.0d;
    wheelPID.SetMode(AUTOMATIC);
  }

  void moveStraight(double linearVelocity) {
    wheelInput = -linearVelocity;
  }

  void turn(double subtendedAngle) {
    wheelInput = wheelRadialOffset * subtendedAngle;
  }

  void moveArc(double radius, double subtendedAngle) {
    wheelInput = -(radius - wheelRadialOffset) * subtendedAngle;
  }

  double getOutput()
  {
    wheelPID.Compute();
    return wheelOutput;
  }

  void stop() {
    wheelPID.SetMode(MANUAL);
  }

};

#endif
