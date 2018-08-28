
#pragma once

#include <Arduino.h>
#include "LighthouseSensor.h"
#include "KVector2.h"
#include "KQuaternion3.h"

class Lighthouse
{

private:
  //sensor above the left wheel
  LighthouseSensor leftSensor;
  //sensor above the right wheel
  LighthouseSensor rightSensor;

  KVector2 previousOrientationVector;
  unsigned long previousOrientationTimeStamp = 0;

  KVector2 orientationVector;
  unsigned long orientationTimeStamp = 0;

  //overall position, which is an average of both sensor positions
  KVector2 previousPositionVector;
  unsigned long previousPositionTimeStamp = 0;

  //overall position, which is an average of both sensor positions
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  //overall velocity
  KVector2 velocityVector;
  unsigned long velocityTimeStamp;

  void setupClock();
  void setupEIC();
  void connectPortPinsToInterrupts();
  void connectInterruptsToTimer();
  void setupTimer();

public:
  Lighthouse();

  void start();
  void loop();

  void clearPreambleFlag() { leftSensor.clearPreambleFlag(); rightSensor.clearPreambleFlag(); }
  bool foundPreamble() { return leftSensor.foundPreamble() && rightSensor.foundPreamble(); }
  bool hasLighthouseSignal() { return leftSensor.hasLighthouseSignal() && rightSensor.hasLighthouseSignal(); }
  void recalculate();
  LighthouseSensor* getLeftSensor() { return &leftSensor; }
  LighthouseSensor* getRightSensor() { return &rightSensor; }

  KVector2* getPosition() { return &positionVector; }
  KVector2* getVelocity() { return &velocityVector; }
  KVector2* getOrientation() { return &orientationVector; }

  void stop();

};
