
#ifndef _KBEZIER2_H_
#define _KBEZIER2_H_

#include "KPathSegment.h"
#include "KVector2.h"

class KBezier2 : public KPathSegment
{
private:
  KVector2 controlPoint;
  KVector2 endPoint;
  double totalLength;

  void recalculateLength();

public:
  KBezier2();
  KBezier2(KVector2* controlPoint, KVector2* endPoint);

  double getEndpointX() { return endPoint.getX(); }
  double getEndpointY() { return endPoint.getY(); }
  double getLength() { return totalLength; }
  void lerpPoint(double atDistance, KVector2* lerpedPoint);
  void set(KVector2* cp, KVector2* ep) {
    set(cp->getX(), cp->getY(), ep->getX(), ep->getY());
  }
  void set(double cx, double cy, double ex, double ey);

};

double lerp(double a1,
            double c1,
            double a2,
            double t);

double lerp(double a1,
            double c1,
            double a2,
            double c2,
            double t);


#endif
