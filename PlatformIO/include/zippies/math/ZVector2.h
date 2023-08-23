
#ifndef _ZVECTOR2_H_
#define _ZVECTOR2_H_

#include <Arduino.h>
#include "ZRotation2.h"

double pad(double value, double epsilon);

class ZVector2
{

protected:
  double x;
  double y;
  mutable double d;
  mutable bool dValid = false;
  mutable double d2;
  mutable bool d2Valid = false;
  mutable double _arctan;
  mutable bool arctanValid = false;
  mutable double _arctan2;
  mutable bool arctan2Valid = false;

public:
  ZVector2();
  ZVector2(const ZVector2* v);
  ZVector2(double x,
           double y);

  void reset();
  void setX(double x);
  double getX() const { return x; }
  void setY(double y);
  double getY() const { return y; }
  void set(const ZVector2* v);
  void set(double x, double y);
  void set(double x, double y, double ofLength);
  // void rotate(double angleRadians);
  void rotate(const ZRotation2* rotation);
  void rotate(double rotation);
  void unrotate(const ZRotation2* rotation);
  void unrotate(double rotation);
  void flip();
  
  double getD() const;
  void setD(double newD);
  double getD2() const;
  double atan() const;
  double atan2() const;
  void normalize() { this->set(this->x, this->y, 1.0); }

  bool equalsVector(const ZVector2* v) const;
  double dotVector(double x2, double y2) const;
  double dotVector(const ZVector2* v) const;
  double dotOrientation(double orientation) const;
  double dotOrientation(const ZRotation2* orientation) const;
  double crossProduct(const ZVector2* v) const;
  double crossProduct(const ZRotation2* v) const;

  void add(const ZVector2* v);
  void add(double x, double y);
  void subtract(const ZVector2* v);
  void subtract(double x, double y);
  void multiply(double factor);
  double projectAlong(double orientation);
  double projectToward(double orientation);
  double angleToVector(const ZVector2* v) const { return angleToOrientation(v->atan2()); }
  double angleToOrientation(double a) const { return subtractAngles(a, atan2()); }

  void printDebug() const;

};

double distanceBetween(double x1, double y1, double x2, double y2);
double distanceBetween(const ZVector2* v1, const ZVector2* v2);
double atanSafe(double x, double y);

#endif
