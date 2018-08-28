
#ifndef _KBEZIER2_H_
#define _KBEZIER2_H_

#include "KPath.h"
#include "KVector2.h"

class KBezier2 : public KPath
{

private:
  const KVector2* controlPoint;
  const KVector2* endPoint;
  double totalLength;
  double finalOrientation;

  void recalculate();

public:
  KBezier2(const KVector2* controlPoint, const KVector2* endPoint);

  void lerp(double atNormalizedTime, KVector2* lerpedPoint) const;
  double getLength() const { return totalLength; }
  double getFinalOrientation() const { return finalOrientation; }

};

double lerp2(double c1,
             double a2,
             double t);

double lerp3(double a1,
             double c1,
             double a2,
             double t);

double lerp4(double a1,
             double c1,
             double a2,
             double c2,
             double t);


#endif
