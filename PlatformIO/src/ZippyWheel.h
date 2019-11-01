#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KMatrix2.h"

#define USE_ZERO_SETPOINT 1

class ZippyWheel
{

private:
  // KVector2 wheelOffset;
  double wheelRadialOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

public:
  ZippyWheel(
    // double wheelOffsetX,
    // double wheelOffsetY,
    double wro,
    double Kp, double Ki, double Kd, double outputLimit,
    unsigned long pidUpdateInterval)
    : //wheelOffset(wheelOffsetX, wheelOffsetY),
      wheelRadialOffset(wro),
      wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, Kp, Ki, Kd, P_ON_E, DIRECT)
  {
    wheelPID.SetSampleTime(pidUpdateInterval);
    wheelPID.SetOutputLimits(-outputLimit, outputLimit);
  }

  void setTunings(double p, double i, double d) {
    wheelPID.SetTunings(p, i, d);
  }

  void start() {
    // wheelPID.SetMode(MANUAL);
    //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
    wheelSetPoint = 0.0d;
    // wheelInput = 0.0d;
    wheelOutput = 0.0d;
    wheelPID.SetMode(AUTOMATIC);
  }

  void setInputStraight(double linearVelocity) {
    wheelInput = linearVelocity;
  }

  void setInputArc(double radius, double subtendedAngle) {
    wheelInput = (radius - wheelRadialOffset) * subtendedAngle;
  }

  double getInput() const { return wheelInput; }

  void moveStraight(double linearVelocity) {
#ifdef USE_ZERO_SETPOINT
    wheelInput = wheelInput - linearVelocity;
#else
    wheelSetPoint = linearVelocity;
#endif
  }

  void turn(double subtendedAngle) {
#ifdef USE_ZERO_SETPOINT
    wheelInput = wheelInput + (wheelRadialOffset * subtendedAngle);
#else
    wheelSetPoint = -wheelRadialOffset * subtendedAngle;
#endif
  }

  void moveArc(double radius, double subtendedAngle) {
#ifdef USE_ZERO_SETPOINT
    wheelInput = wheelInput - ((radius - wheelRadialOffset) * subtendedAngle);
#else
    wheelSetPoint = (radius - wheelRadialOffset) * subtendedAngle;
#endif
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
