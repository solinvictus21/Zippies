#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

#include <PID_v1.h>
#include "zippies/ZippyMath.h"
#include "zippies/config/PIDConfig.h"

class ZippyWheel
{

private:
  double wheelRadialOffset;
  double wheelAngleOffset;

  double wheelInput = 0.0d;
  double wheelSetPoint = 0.0d;
  double wheelOutput = 0.0d;
  PID wheelPID;

public:
  ZippyWheel(double wro, double wao)
    : wheelRadialOffset(wro),
      wheelAngleOffset(wao),
      wheelPID(
        &wheelInput, &wheelOutput, &wheelSetPoint,
        PID_KP, PID_KI, PID_KD, P_ON_E, DIRECT)
  {
    /*
    SerialUSB.print("PID: ");
    SerialUSB.print(PID_KP, 2);
    SerialUSB.print(", ");
    SerialUSB.print(PID_KI, 2);
    SerialUSB.print(", ");
    SerialUSB.print(PID_KD, 2);
    SerialUSB.print(", ");
    SerialUSB.print(PID_UPDATE_INTERVAL);
    SerialUSB.print(", ");
    SerialUSB.print(PID_OUTPUT_LIMIT, 2);
    SerialUSB.println();
    */
    wheelPID.SetSampleTime(PID_UPDATE_INTERVAL);
    wheelPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  }

  void setTunings(double p, double i, double d) {
    // wheelPID.SetTunings(p, i, d);
  }

  void start() {
    wheelSetPoint = 0.0d;
    wheelInput = 0.0d;
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
