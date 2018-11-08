
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/KPosition.h"
#include "ZippyWheel.h"

// #define INDEPENDENT_WHEEL_PIDS 1

class Zippy
{

private:
  ZippyFace face;
  MotorDriver motors;

  unsigned long pidUpdateInterval;
  bool inReverse = false;

  KPosition currentTargetPosition;
  bool positionUpdated = false;
  bool orientationUpdated = false;
  bool prioritizeOrientation = false;
  bool stopped = false;

#ifdef INDEPENDENT_WHEEL_PIDS
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
#else
  double angularInput = 0.0d;
  double angularSetPoint = 0.0d;
  double angularOutput = 0.0d;
  PID angularPID;
  double linearInput = 0.0d;
  double linearSetPoint = 0.0d;
  double linearOutput = 0.0d;
  PID linearPID;
#endif

  void setMotors(int32_t motorLeft, int32_t motorRight);

public:
  Zippy(unsigned long pidUpdateInterval);

  ZippyFace* getFace() { return &face; }

  void start();

  void setReverse(bool r) { inReverse = r; }
  void move(double x, double y, double orientation);
  void turn(double orientation);
  void turnAndMove(double x, double y, double orientation);
  const KPosition* getTargetPosition() { return &currentTargetPosition; }

  bool loop(const KPosition* currentPosition,
    const KPosition* currentVelocity);
  void updateInputs(double relativeVelocityDistance, double relativeOrientation);

  void stop();
  bool isStopped() { return stopped; }

};

#endif
