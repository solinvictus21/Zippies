
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/KMatrix2.h"
#include "lighthouse/KRotation2.h"
#include "ZippyConfig.h"
#include "ZippyWheel.h"
#include "paths/ZPath.h"
#include "ZippyRoutine.h"

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
  PreparingToMove
} MovementState;

class Zippy
{

private:
#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  KMatrix2 currentPosition;
  // KMatrix2 currentVelocity;
  MovementState currentMovementState = MovementState::Stopped;

  KMatrix2 targetPosition;
  bool targetPositionUpdated = false;
  bool targetOrientationUpdated = false;
  KMatrix2 targetVelocity;

  ZippyWheel leftWheel;
  ZippyWheel rightWheel;
  MotorDriver motors;

  bool errorCaptureEnabled = false;
  double errorM = 0.0d;
  double errorS = 0.0d;
  unsigned long errorCounter = 0;

  friend class ZPrimaryController;

  void start();
  // void setInputs(const KMatrix2* cp, const KMatrix2* cv);
  void setCurrentPosition(const KMatrix2* p);
  void loop();
  // void processInputs();
  void captureError();
  void executeMove();
  void executeLinearMove();
  void executeTurn();
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
