
#ifndef _ZIPPYCONTROLLER_H_
#define _ZIPPYCONTROLLER_H_

#include "lighthouse/KPosition.h"

class ZippyController
{

protected:
  ZippyController() {}

public:

  virtual const KPosition* getTargetPosition() const = 0;
  virtual void waitForPreamble() = 0;
  virtual bool foundPreamble() const = 0;
  virtual void startMoving() = 0;
  virtual void move(double x, double y, double orientation) = 0;
  virtual void turn(double orientation) = 0;
  virtual void stopMoving() = 0;

};

#endif
