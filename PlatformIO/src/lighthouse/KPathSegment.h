
#ifndef _KPATHSEGMENT_H_
#define _KPATHSEGMENT_H_

class KVector2;

class KPathSegment
{

public:
  virtual double getLength() = 0;
  virtual double getEndpointX() = 0;
  virtual double getEndpointY() = 0;
  virtual void lerpPoint(double distance, KVector2* lerpedPoint) = 0;

  virtual ~KPathSegment() {};

};

#endif
