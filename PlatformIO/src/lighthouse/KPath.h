
#ifndef _KPATHSEGMENT_H_
#define _KPATHSEGMENT_H_

class KVector2;

class KPath
{

public:
  virtual void lerp(double atNormalizedTime, KVector2* lerpedPoint) const = 0;
  virtual double getLength() const = 0;

  virtual ~KPath() {};

};

#endif
