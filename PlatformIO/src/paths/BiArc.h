
#ifndef _BIARC_H_
#define _BIARC_H_

#include "ZPath.h"

class BiArc : public ZPath
{
private:
  const Arc* arc1;
  const Arc* arc2;
  double totalArcLength;

public:
  BiArc(const Arc* a1, const Arc* a2)
    : arc1(a1),
      arc2(a2)
  {
    totalArcLength = arc1->getLength() + arc2->getLength();
  }

  double getLength() const { return totalArcLength; }

  void interpolate(double normalizedTime, KPosition* targetPosition) const
  {
    double distance = normalizedTime * totalArcLength;
    double arcLength1 = arc1->getLength();
    if (distance < arcLength1) {
      //moving through first arc
      arc1->interpolate(distance / arcLength1, targetPosition);
    }
    else {
      //moving through second arc
      distance -= arcLength1;
      arc2->interpolate(distance / arc2->getLength(), targetPosition);
    }
  }

  ~BiArc()
  {
    delete arc1;
    delete arc2;
  }

};

#endif
