
#ifndef _LIGHTHOUSE_H_
#define _LIGHTHOUSE_H_

#include <Arduino.h>
#include "LighthouseSensor.h"
#include "KVector2.h"
#include "KQuaternion3.h"
#include "KPosition.h"

//#define DEBUG_SIGNAL_EDGES 1

class Lighthouse
{

private:
  //sensors
  LighthouseSensor leftSensor;
  LighthouseSensor rightSensor;

  KPosition previousPosition;
  unsigned long previousPositionTimeStamp = 0;

  KPosition position;
  KPosition positionDelta;
  unsigned long positionTimeStamp = 0;

  unsigned long lostPositionTimeStamp = 0;

  void setupClock();
  void setupEIC();
  void connectPortPinsToInterrupts();
  void connectInterruptsToTimer();
  void setupTimer();

  void calculatePosition();
  void calculateVelocity();

public:
  Lighthouse();

  void start();
  bool loop(unsigned long currentTime);

  bool recalculate(unsigned long currentTime);
  void estimatePosition(unsigned long currentTime);

  void clearPreambleFlag() { leftSensor.clearPreambleFlag(); rightSensor.clearPreambleFlag(); }
  bool foundPreamble() { return leftSensor.foundPreamble() && rightSensor.foundPreamble(); }

  const LighthouseSensor* getLeftSensor() const { return &leftSensor; }
  const LighthouseSensor* getRightSensor() const { return &rightSensor; }
  const KPosition* getPosition() const { return &position; }
  const KPosition* getPositionDelta() const { return &positionDelta; }

  bool isSignalLocked() { return leftSensor.isSignalLocked() && rightSensor.isSignalLocked(); }

  void stop();

};

#endif
