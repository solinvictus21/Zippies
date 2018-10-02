
#ifndef _LIGHTHOUSE_H_
#define _LIGHTHOUSE_H_

#include <Arduino.h>
#include "LighthouseSensor.h"
#include "KVector2.h"
#include "KQuaternion3.h"
#include "KPosition.h"

class Lighthouse
{

private:
  //sensors
  LighthouseSensor leftSensor;
  LighthouseSensor rightSensor;

  KPosition previousPosition;
  KPosition previousPositionDelta;
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
  void estimatePosition(unsigned long currentTime);
  void calculateVelocity();

public:
  Lighthouse();

  void start();
  void loop(unsigned long currentTime);

  bool recalculate(unsigned long currentTime);

  void clearPreambleFlag() { leftSensor.clearPreambleFlag(); rightSensor.clearPreambleFlag(); }
  bool foundPreamble() { return leftSensor.foundPreamble() && rightSensor.foundPreamble(); }

  const KPosition* getPosition() const { return &position; }
  const KPosition* getPositionDelta() const { return &positionDelta; }

  void stop();

};

#endif
