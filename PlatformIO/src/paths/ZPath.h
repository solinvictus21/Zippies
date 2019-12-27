
#ifndef _ZPATH_H_
#define _ZPATH_H_

#include "zippies/ZippyMath.h"

class ZPath
{

protected:
  ZPath() {}

public:
  virtual bool updatesPosition() const { return false; }
  // virtual double getLength() const = 0;

  virtual void interpolate(
    double interpolatedTime,
    KMatrix2* targetPosition) const = 0;

  virtual ~ZPath() {}

};

#endif
