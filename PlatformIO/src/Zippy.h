
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "ZippyWheel.h"
#include "MotorDriver.h"
#include "lighthouse/KPosition.h"

// #define MOTOR_MODEL_COMBINED 1

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

#ifdef MOTOR_MODEL_COMBINED
  double linearSetPoint = 0.0d;
  double linearInput = 0.0d;
  double linearOutput = 0.0d;
  PID linearPID;

  double angularSetPoint = 0.0d;
  double angularInput = 0.0d;
  double angularOutput = 0.0d;
  PID angularPID;

  bool calculateLinearInput(const KVector2* deltaPosition,
                            const KPosition* currentPosition,
                            const KPosition* currentVelocity);
  bool calculateAngularInput(const KVector2* deltaPosition,
                             const KPosition* currentPosition,
                             const KPosition* currentVelocity);
#else
  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
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

  void stop();
  bool isStopped() { return stopped; }

};

#endif
