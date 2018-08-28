
#ifndef _LINEPATH_H_
#define _LINEPATH_H_

#include "../lighthouse/KPath.h"
#include "../lighthouse/KVector2.h"

class LinePath : public KPath
{

private:
  const KVector2* startingPosition;
  const KVector2* endingPosition;
  double length;

public:
  LinePath(const KVector2* sp, const KVector2* ep)
    : startingPosition(sp),
      endingPosition(ep)
  {
    length = sqrt(pow(endingPosition->getX() - startingPosition->getX(), 2.0d) +
        pow(endingPosition->getY() - startingPosition->getY(), 2.0d));
  }

  void lerp(double atNormalizedTime, KVector2* lerpedPoint) const {
    lerpedPoint->set(startingPosition->getX() + ((endingPosition->getX() - startingPosition->getX()) * atNormalizedTime),
        startingPosition->getY() + ((endingPosition->getY() - startingPosition->getY()) * atNormalizedTime));
  }

  double getLength() const { return length; }

  double getFinalOrientation() const { return atan2(endingPosition->getX() - startingPosition->getX(),
      endingPosition->getY() - startingPosition->getY()); }

};

#endif
