
#ifndef _ZIPPY_H_
#define _ZIPPY_H_

#include "Platform.h"
#include "ZippyFace.h"
#include "lighthouse/Lighthouse.h"
#include "MotorDriver.h"

class Zippy
{

private:
  ZippyFace face;
  Lighthouse lighthouse;
  MotorDriver motors;

public:
  Zippy();

  void start();
  void loop();

  Lighthouse* getLighthouse() { return &lighthouse; }
  bool hasLighthouseSignal() { return lighthouse.hasLighthouseSignal(); }
  void recalculate() { lighthouse.recalculate(); }
  KVector2* getPosition() { return lighthouse.getPosition(); }
  double getVelocity() { return lighthouse.getLinearVelocity(); }
  KVector2* getOrientation() { return lighthouse.getOrientation(); }
  void setMotors(int32_t motorLeft, int32_t motorRight);
  void stop() { setMotors(0.0d, 0.0d); }

};

#endif
