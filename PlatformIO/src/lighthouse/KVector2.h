
#ifndef _KVECTOR2_H_
#define _KVECTOR2_H_

#include <Arduino.h>
#include "KPath.h"

extern double subtractAngles(double a1, double a2);
extern double addAngles(double a1, double a2);

class KVector2 : public KPath
{

protected:
  double x;
  double y;
  mutable double d, d2;
  mutable bool dValid, d2Valid;
  mutable double orientation;
  mutable bool orientationValid;

  void setD(double newD, KVector2* unitVector);

public:
  KVector2();
  KVector2(KVector2* v);
  KVector2(double x,
           double y);
  KVector2(double x,
           double y,
           double ofLength);

  void reset();
  void setX(double x);
  double getX() const { return x; }
  void setY(double y);
  double getY() const { return y; }
  void set(KVector2* v);
  void set(double x, double y);
  void set(double x, double y, double ofLength);
  void rotate(double angleRadians);

  double getD() const;
  void setD(double newD);
  double getD2() const;
  double getOrientation() const;
  void normalize() { this->setD(1.0f); }

  bool equalsVector(KVector2* v) const;
  double dotVector(KVector2* v) const;

  void addVector(KVector2* v);
  void subtractVector(KVector2* v);
  double angleToVector(KVector2* v) const { return angleToOrientation(v->getOrientation()); }
  double angleToOrientation(double a) const { return subtractAngles(a, getOrientation()); }

  void printDebug() const;

  /*
  double getEndpointX() { return x; }
  double getEndpointY() { return x; }
  double getLength() { return getD(); }
  */
  void lerp(double atNormalizedTime, KVector2* lerpedPoint) const;
  double getLength() const { return getD(); }

};

#endif
