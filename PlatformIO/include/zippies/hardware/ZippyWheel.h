#ifndef _ZIPPYWHEEL_H_
#define _ZIPPYWHEEL_H_

// #include <PID_v1.h>
#include "zippies/math/PID_v2.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/PIDConfig.h"

class ZippyWheel
{

private:
  KVector2 wheelOffset;
  double directTurnFactor;

  PID_v2 wheelPID;

public:
  ZippyWheel(double wox, double woy)
    : wheelOffset(wox, woy),
      directTurnFactor(1.0d + sin(atan(woy / wox))),
      wheelPID(60, 0.0d,
        PID_KP, PID_KI, PID_KD,
        -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
        false, false)
  {
  }

  void setTunings(double p, double i, double d) {
    wheelPID.setTunings(p, i, d);
  }

  void start() {
    wheelPID.start();
  }

  double moveStraight(double linearVelocity) {
    return wheelPID.compute(-linearVelocity);
  }

  double turn(double subtendedAngle) {
    return wheelPID.compute(wheelOffset.getX() * subtendedAngle);
    // return wheelPID.compute(directTurnFactor * wheelOffset.getX() * subtendedAngle);
  }

  double moveArc(double radius, double subtendedAngle) {
    return wheelPID.compute(-(radius - wheelOffset.getX()) * subtendedAngle);
    /*
    double wheelRadius = radius - wheelOffset.getX();
    if (wheelRadius == 0.0d)
      return wheelPID.compute(0.0d);

    // SerialUSB.print(wheelOffset.getX() < 0.0d ? "Left " : "Right ");
    // SerialUSB.print(wheelRadius, 5);
    // SerialUSB.print(" ");
    // double wheelD2 = pow(invWheelRadius,2.0d)+pow(wheelOffset.getY(),2.0d);
    // SerialUSB.println(1.0d + sin(atan(-wheelOffset.getY() / -(radius + wheelOffset.getX()))), 5);

    // double wheelInput = -wheelRadius * subtendedAngle;
    double wheelInput = wheelRadius * subtendedAngle;
    wheelInput *= 1.0d + sin(atan(-wheelOffset.getY() / wheelRadius));
    return wheelPID.compute(-wheelInput);
    */
  }

  void stop() {
    wheelPID.stop();
  }

};

#endif
