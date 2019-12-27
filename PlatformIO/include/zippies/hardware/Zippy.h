
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "zippies/ZippyHardware.h"
#include "zippies/ZippyMath.h"
#include "zippies/config/BodyConfig.h"
#include "zippies/math/StatisticsAccumulator.h"
#include "ZippyWheel.h"
#include "paths/ZPath.h"

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
} MovementState;

class Zippy
{

private:
#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  KMatrix2 currentPosition;
  MovementState currentMovementState = MovementState::Stopped;

  KMatrix2 targetPosition;
  bool targetPositionUpdated = false;
  bool targetOrientationUpdated = false;

  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  MotorDriver motors;

  bool errorCaptureEnabled = false;
  StatisticsAccumulator statisticsAccumulator;

  friend class TargetController;

  void start();
  void setCurrentPosition(const KMatrix2* p);
  void loop();
  void executeMove(KMatrix2* targetVelocity);
  void executeLinearMove(double linearVelocity);
  void executeTurn(double angle);
  void stop();

public:
  Zippy();

  //for auto-tuning; only call this while the Zippy is stopped
  void setTunings(double p, double i, double d) {
    leftWheel.setTunings(p, i, d);
    rightWheel.setTunings(p, i, d);
  }

  void setTargetPosition(const KMatrix2* tp);
  void setTargetOrientation(const KRotation2* r);
  bool isStopped() { return currentMovementState == MovementState::Stopped; }

  ZippyFace* getFace() { return &face; }
  void startErrorCapture();
  void stopErrorCapture();
  double getStandardDeviation() const;

};

#endif
