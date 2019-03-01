
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "ZippyFace.h"
#include "MotorDriver.h"
#include "lighthouse/KPosition.h"
#include "ZippyConfig.h"
#ifdef KINEMATIC_MODEL_INDEPENDENT
#include "ZippyWheel.h"
#endif

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#define SD_CHIP_SELECT 10
#endif

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
  bool prioritizeOrientation = false;

#ifdef KINEMATIC_MODEL_INDEPENDENT
  double currentTargetVelocity = 0.0d;
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

#ifdef ENABLE_SDCARD_LOGGING
  // SPIFlash flash(5);
  bool loggingEnabled = false;
#endif

  void setMotors(int32_t motorLeft, int32_t motorRight);
  void driveStop();
  void driveTurn(double relativeTargetOrientation);
  // void driveNear(KVector2* relativeTargetPosition, double relativeDirectionOfMotion);
  // void driveFar(KVector2* relativeTargetPosition, double relativeDirectionOfMotion);
  // void driveNormal(KVector2* relativeTargetPosition);
  void driveCurved(KVector2* relativeTargetPosition, double relativeTargetOrientation, double relativeDirectionOfMotion);
  void drivePinned(KVector2* relativeTargetPosition, double relativeTargetOrientation);
  void driveMotors();
  bool isFacingWrongDirection(double directionOfMotion);

public:
  Zippy(unsigned long pidUpdateInterval);

#ifdef PLATFORM_TINYSCREEN
  ZippyFace* getFace() { return &face; }
#endif

  void start();

  void setReverse(bool r) { inReverse = r; }
  bool isInReverse() { return inReverse; }
  void move(double x, double y, double orientation);
  void turn(double orientation);
  void turnAndMove(double x, double y, double orientation);
  const KPosition* getTargetPosition() const { return &currentTargetPosition; }
  const double getTargetVelocity() const { return currentTargetVelocity; }

  bool loop(const KPosition* currentPosition,
    const KPosition* currentVelocity);
  void stop() { setMotors(0.0d, 0.0d); }

  const ZippyWheel* getLeftWheel() const { return &leftWheel; }
  const ZippyWheel* getRightWheel() const { return &rightWheel; }

#ifdef ENABLE_SDCARD_LOGGING
  void startLogging();
  void stopLogging();
#endif

};

#endif
