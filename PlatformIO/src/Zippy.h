
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include <PID_v1.h>
#include "commands/ZippyCommand.h"
#include "ZippyFace.h"
#include "lighthouse/Lighthouse.h"
#include "MotorDriver.h"

#define ZIPPY_COMMAND_COUNT 5

class Zippy
{

private:
  ZippyFace face;
  Lighthouse lighthouse;
  MotorDriver motors;
  ZippyCommand* commands[ZIPPY_COMMAND_COUNT];

  int currentCommand;
  unsigned long currentCommandStartTime = 0;
  KVector2 targetPosition;
  double targetOrientation;

  bool moving;

  double linearSetPoint = 0.0d;
  double linearInput = 0.0d;
  double linearOutput = 0.0d;
  PID linearPID;

  double rotationalSetPoint = 0.0d;
  double rotationalInput = 0.0d;
  double rotationalOutput = 0.0d;
  PID rotationalPID;

  unsigned long lostPositionTimestamp = 0;
  unsigned long lastUpdateTime = 0;

  void calculateVelocityInput(KVector2* deltaPosition);
  void setMotors(int32_t motorLeft, int32_t motorRight);

public:
  Zippy();

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);

  Lighthouse* getLighthouse() { return &lighthouse; }
  void stop() { setMotors(0, 0); }
  // bool hasLighthouseSignal() { return lighthouse.hasLighthouseSignal(); }
  // void recalculate() { lighthouse.recalculate(); }
  // KVector2* getPosition() { return lighthouse.getPosition(); }
  // KVector2* getVelocity() { return lighthouse.getVelocity(); }
  // KVector2* getOrientation() { return lighthouse.getOrientation(); }

  ~Zippy();

};

#endif
