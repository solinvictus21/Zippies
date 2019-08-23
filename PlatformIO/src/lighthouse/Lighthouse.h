
#ifndef _LIGHTHOUSE_H_
#define _LIGHTHOUSE_H_

#include <Arduino.h>
#include "LighthouseSensor.h"
#include "KVector2.h"
#include "KQuaternion3.h"
#include "KMatrix2.h"

//#define DEBUG_SIGNAL_EDGES 1

class Lighthouse
{

private:
  //sensors
  LighthouseSensor leftSensor;
  LighthouseSensor rightSensor;

  KMatrix2 previousPosition;
  unsigned long previousPositionTimeStamp = 0;

  KMatrix2 position;
  KMatrix2 positionDelta;
  unsigned long positionTimeStamp = 0;

  unsigned long positionLockedTimeStamp = 0;
  unsigned long positionUnlockedTimeStamp = 0;
  bool positionValid = false;

  void setupClock();
  void setupEIC();
  void connectPortPinsToInterrupts();
  void connectInterruptsToTimer();
  void setupTimer();

public:
  Lighthouse();

  void start();
  bool loop(unsigned long currentTime);
  bool recalculate();

  bool recalculate(unsigned long currentTime);
  void estimatePosition(unsigned long currentTime);

  void clearPreambleFlag() { leftSensor.clearPreambleFlag(); rightSensor.clearPreambleFlag(); }
  bool foundPreamble() const { return leftSensor.foundPreamble() && rightSensor.foundPreamble(); }

  const LighthouseSensor* getLeftSensor() const { return &leftSensor; }
  const LighthouseSensor* getRightSensor() const { return &rightSensor; }
  const KMatrix2* getPosition() const { return &position; }
  const KMatrix2* getPositionDelta() const { return &positionDelta; }

  bool isSignalLocked() { return leftSensor.isSignalLocked() && rightSensor.isSignalLocked(); }

  void stop();

};

#endif
