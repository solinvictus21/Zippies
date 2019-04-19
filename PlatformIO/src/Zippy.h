
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/KPosition.h"
#include "ZippyConfig.h"
#include "ZippyWheel.h"

class Zippy
{

private:
#ifdef PLATFORM_TINYSCREEN
  ZippyFace face;
#endif
  MotorDriver motors;

  bool inReverse = false;

  KPosition currentTargetPosition;
  bool positionUpdated = false;
  bool orientationUpdated = false;

  ZippyWheel leftWheel;
  ZippyWheel rightWheel;

  void plotBiArc(KVector2* relativeTargetPosition, double relativeDirectionOfMotion,
    double* linearVelocity, double* angularVelocity);

  void setMotors(int32_t motorLeft, int32_t motorRight);
  void driveStop();
  void driveArc(double linearVelocity, double angularVelocity);
  // void driveTurn(double relativeTargetOrientation);
  // void driveCurved(KVector2* relativeTargetPosition, double relativeTargetOrientation, double relativeDirectionOfMotion);
  // void driveWheels(double linearVelocity, double angularVelocity);
  // void drivePinned(KVector2* relativeTargetPosition, double relativeTargetOrientation);
  // void driveMotors();

  double saturate(double a, double b);
  double centerTurnRadius(double distanceDelta, double orientationDelta);
  double rampUp(double velocity);
  // double constrainAcceleration(double previousValue, double newValue, double changeFactor, double minimumAcceleration);

public:
  Zippy(unsigned long pidUpdateInterval);

#ifdef PLATFORM_TINYSCREEN
  ZippyFace* getFace() { return &face; }
#endif

  void start();

  void setReverse(bool r) { inReverse = r; }
  bool isInReverse() { return inReverse; }
  // void move(double x, double y);
  void move(double x, double y, double orientation);
  void turn(double orientation);
  const KPosition* getTargetPosition() const { return &currentTargetPosition; }

  bool loop(const KPosition* currentPosition,
    const KPosition* currentVelocity);
  void stop() { setMotors(0.0d, 0.0d); }

  const ZippyWheel* getLeftWheel() const { return &leftWheel; }
  const ZippyWheel* getRightWheel() const { return &rightWheel; }
};

#endif
