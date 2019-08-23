#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "lighthouse/KVector2.h"
#include "lighthouse/KMatrix2.h"

#define DEBUG_PID_ERROR

#define COS_WHEEL 0.942886053042126d

class ZippyWheel
{

private:
  KVector2 wheelOffset;
  // double wheelRadialOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

  #ifdef DEBUG_PID_ERROR
      double accumulatingError = 0.0d;
      int cycleCount = 0;
      double wheelMaxOutput = 0;
  #endif

public:
  ZippyWheel(
    double wheelOffsetX,
    double wheelOffsetY,
    double Kp, double Ki, double Kd, double outputLimit,
    unsigned long pidUpdateInterval)
    : wheelOffset(wheelOffsetX, wheelOffsetY),
      wheelPID(&wheelInput, &wheelOutput, &wheelSetPoint, Kp, Ki, Kd, P_ON_E, DIRECT)
  {
    // wheelRadialOffset = wheelOffset.getX();
    // wheelRadialOffset = 2.0d * wheelOffset.getD();
    // if (wheelOffset.getX() < 0.0d)
      // wheelRadialOffset = -wheelRadialOffset;
    wheelPID.SetSampleTime(pidUpdateInterval);
    wheelPID.SetOutputLimits(-outputLimit, outputLimit);
  }

  void start() {
    wheelPID.SetMode(MANUAL);
    //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
    wheelSetPoint = 0.0d;
    wheelInput = 0.0d;
    wheelOutput = 0.0d;
    wheelPID.SetMode(AUTOMATIC);
  }

  void setInputStraight(double linearVelocity) {
    wheelInput = linearVelocity;

#ifdef DEBUG_PID_ERROR
    accumulatingError += wheelInput - wheelSetPoint;
    cycleCount++;
#endif
  }

  void setInputArc(double radius, double subtendedAngle) {
    wheelInput = (radius - wheelOffset.getX()) * subtendedAngle;
    // if (radius * wheelOffset.getX() <= 0.0d)
      // wheelInput /= COS_WHEEL;

#ifdef DEBUG_PID_ERROR
    accumulatingError += wheelInput - wheelSetPoint;
    cycleCount++;
#endif
  }

  void turn(double subtendedAngle) {
    wheelSetPoint = -wheelOffset.getX() * subtendedAngle;
    // wheelSetPoint /= COS_WHEEL;
    wheelPID.Compute();

#ifdef DEBUG_PID_ERROR
    wheelMaxOutput = max(wheelMaxOutput, abs(wheelOutput));
#endif
  }

  void moveStraight(double linearVelocity) {
    wheelSetPoint = linearVelocity;
    wheelPID.Compute();

#ifdef DEBUG_PID_ERROR
    wheelMaxOutput = max(wheelMaxOutput, abs(wheelOutput));
#endif
  }

  void moveArc(double radius, double subtendedAngle) {
    wheelSetPoint = (radius - wheelOffset.getX()) * subtendedAngle;
    // if (radius * wheelOffset.getX() <= 0.0d)
      // wheelSetPoint /= COS_WHEEL;
    wheelPID.Compute();

#ifdef DEBUG_PID_ERROR
    wheelMaxOutput = max(wheelMaxOutput, abs(wheelOutput));
#endif
  }

  void stop() {
    wheelSetPoint = 0.0d;
    wheelPID.Compute();

#ifdef DEBUG_PID_ERROR
    wheelMaxOutput = max(wheelMaxOutput, abs(wheelOutput));
#endif
  }

  double getSetPoint() const { return wheelSetPoint; }

  double getOutput() const { return wheelOutput; }

#ifdef DEBUG_PID_ERROR
  double getAverageError() {
    return cycleCount == 0 ? 0.0d : accumulatingError / ((double)cycleCount);
  }

  double getMaxOutput() {
    return wheelMaxOutput;
  }

  void clearError() {
    accumulatingError = 0.0d;
    cycleCount = 0;
  }
#endif

};

#endif
