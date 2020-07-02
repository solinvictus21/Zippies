
#ifndef _PATHSEGMENT_H_
#define _PATHSEGMENT_H_

#include "zippies/math/ZMatrix2.h"

class PathSegment
{

protected:
  PathSegment() {}

public:
  // virtual bool updatesPosition() const { return false; }
  // virtual double getLength() const = 0;

  virtual void interpolate(
    double interpolatedTime,
    ZMatrix2* targetPosition) const = 0;

  virtual ~PathSegment() {}

};

#endif
