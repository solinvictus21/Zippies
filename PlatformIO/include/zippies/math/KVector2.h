
#ifndef _KVECTOR2_H_
#define _KVECTOR2_H_

#include <Arduino.h>
#include "KRotation2.h"

class KVector2
{

protected:
  double x;
  double y;
  mutable double d;
  mutable bool dValid;
  mutable double d2;
  mutable bool d2Valid;
  mutable double _arctan;
  mutable bool arctanValid;
  mutable double _arctan2;
  mutable bool arctan2Valid;

public:
  KVector2();
  KVector2(const KVector2* v);
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
  void set(const KVector2* v);
  void set(double x, double y);
  void set(double x, double y, double ofLength);
  void rotate(double angleRadians);
  void rotate(const KRotation2* rotation);
  void unrotate(const KRotation2* rotation);

  double getD() const;
  void setD(double newD);
  double getD2() const;
  double atan() const;
  double atan2() const;
  void normalize() { this->set(this->x, this->y, 1.0d); }

  bool equalsVector(const KVector2* v) const;
  double dotVector(const KVector2* v) const;
  double dotOrientation(double orientation) const;
  double crossProduct(const KVector2* v) const;

  void addVector(const KVector2* v);
  void subtractVector(const KVector2* v);
  void multiply(double factor);
  double projectAlong(double orientation);
  double projectToward(double orientation);
  double angleToVector(const KVector2* v) const { return angleToOrientation(v->atan2()); }
  double angleToOrientation(double a) const { return subtractAngles(a, atan2()); }

  void printDebug() const;

};

double distanceBetween(double x1, double y1, double x2, double y2);
double distanceBetween(const KVector2* v1, const KVector2* v2);

#endif
