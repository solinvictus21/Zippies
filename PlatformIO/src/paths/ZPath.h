
#ifndef _ZPATH_H_
#define _ZPATH_H_

#include "../lighthouse/KPosition.h"

class ZPath
{

protected:
  ZPath() {}

public:
  virtual double getLength() const = 0;
  virtual void interpolate(
    double interpolatedTime,
    KPosition* targetPosition,
    bool* reverseMotion) const = 0;

  virtual ~ZPath() {}

};

#endif
