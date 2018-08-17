
#ifndef _KVECTOR2_H_
#define _KVECTOR2_H_

#include "KPathSegment.h"

class KVector2 : public KPathSegment
{
protected:
  double x;
  double y;
  double d, d2;
  bool dValid, d2Valid;
  double orientation;
  bool orientationValid;

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
  double getX() { return x; }
  void setY(double y);
  double getY() { return y; }
  void set(KVector2* v);
  void set(double x, double y);
  void set(double x, double y, double ofLength);
  void rotate(double angleRadians);

  double getD();
  double getD2();
  double getOrientation();
  void setD(double newD);
  void normalize() { this->setD(1.0f); }

  bool equalsVector(KVector2* v);
  double dotVector(KVector2* v);

  void addVector(KVector2* v);
  void subtractVector(KVector2* v);
  double angleToVector(KVector2* v);

  void printDebug();

  double getEndpointX() { return x; }
  double getEndpointY() { return x; }
  double getLength() { return getD(); }
  void lerpPoint(double atDistance, KVector2* lerpedPoint);

};

#endif
