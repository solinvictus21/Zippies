
#include <SPI.h>
#include "KVector2.h"

#define EPSILON 0.0001d

KVector2::KVector2()
{
  reset();
}

KVector2::KVector2(const KVector2* v)
  : x(v->x),
    y(v->y),
    d(v->d),
    dValid(v->dValid),
    d2(v->d2),
    d2Valid(v->d2Valid),
    orientation(v->orientation),
    orientationValid(v->orientationValid)
{
}

KVector2::KVector2(double x,
                   double y)
{
    this->x = x;
    this->y = y;
    d2Valid = false;
    dValid = false;
    orientationValid = false;
}

KVector2::KVector2(double x,
                   double y,
                   double ofLength)
{
  set(x, y, ofLength);
}

double KVector2::getD() const
{
  if (!dValid) {
      d = sqrt(this->getD2());
      dValid = true;
  }
  return d;
}

double KVector2::getD2() const
{
  if (!d2Valid) {
      if (dValid)
          d2 = d*d;
      else
          d2 = this->x*this->x + this->y*this->y;
      d2Valid = true;
  }

  return d2;
}

bool KVector2::equalsVector(const KVector2* v) const
{
  return fabs(v->x-x) < EPSILON && fabs(v->y-y) < EPSILON;
}

/**
 * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
 * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
 * result will be just cos(ϴ).
 */
double KVector2::dotVector(const KVector2* v) const
{
  return (x*v->x) + (y*v->y);
}

double KVector2::dotOrientation(double o) const
{
  return (x * sin(o)) + (y * cos(o));
}

void KVector2::multiply(double m)
{
  x *= m;
  y *= m;
  if (dValid)
    d *= m;
  d2Valid = false;
}

void KVector2::reset()
{
  x = 0.0d;
  y = 0.0d;
  d = 0.0d;
  dValid = true;
  d2 = 0.0d;
  d2Valid = true;
  orientation = 0.0d;
  orientationValid = true;
}

void KVector2::setX(double newX)
{
    if (x == newX)
        return;

    x = newX;
    dValid = false;
    d2Valid = false;
    orientationValid = false;
}

void KVector2::setY(double newY)
{
    if (y == newY)
        return;

    y = newY;
    dValid = false;
    d2Valid = false;
    orientationValid = false;
}

void KVector2::set(const KVector2* v)
{
  this->set(v->x, v->y);
}

void KVector2::set(double newX,
                   double newY)
{
    x = newX;
    y = newY;
    dValid = false;
    d2Valid = false;
    orientationValid = false;
}

void KVector2::set(double newX,
                   double newY,
                   double ofLength)
{
  if (ofLength == 0.0d || (newX == 0.0d && newY == 0.0d)) {
    this->x = 0.0d;
    this->y = 0.0d;
    d = 0.0d;
    dValid = true;
    d2 = 0.0d;
    d2Valid = true;
    orientation = 0.0d;
    orientationValid = true;
    return;
  }

  //calculate the actual values from the unit vector
  double vd = sqrt(newX*newX+newY*newY);
  this->x = (newX*ofLength)/vd;
  this->y = (newY*ofLength)/vd;
  d = ofLength;
  dValid = true;
  d2 = d*d;
  d2Valid = true;
  orientationValid = false;
}

void KVector2::setD(double newD)
{
  set(this->x, this->y, newD);
  // double lengthRatio = newD / getD();
  // this->x *= lengthRatio;
  // this->y *= lengthRatio;
}

void KVector2::addVector(const KVector2* v)
{
  this->x += v->x;
  this->y += v->y;
  dValid = false;
  d2Valid = false;
  orientationValid = false;
}

void KVector2::subtractVector(const KVector2* v)
{
  this->x -= v->x;
  this->y -= v->y;
  dValid = false;
  d2Valid = false;
  orientationValid = false;
}

double KVector2::projectAlong(double orientation)
{
  double sinTheta = sin(orientation);
  double cosTheta = cos(orientation);
  double dotProduct = (x * sinTheta) + (y * cosTheta);
  this->x = dotProduct * sinTheta;
  this->y = dotProduct * cosTheta;
  double length = dotProduct / cosTheta;
  this->d = abs(length);
  dValid = true;
  //the result could be either the orientation specified or +M_PI, exactly the opposite direction
  this->orientation = dotProduct >= 0.0d ? orientation : addAngles(orientation, M_PI);
  orientationValid = true;
  d2Valid = false;

  return length;
}

double KVector2::projectToward(double orientation)
{
  double sinTheta = sin(orientation);
  double cosTheta = cos(orientation);
  double dotProduct = (x * sinTheta) + (y * cosTheta);
  double absDotProduct = abs(dotProduct);
  this->x = absDotProduct * sinTheta;
  this->y = absDotProduct * cosTheta;
  double length = dotProduct / cosTheta;
  this->d = abs(length);
  dValid = true;
  // dValid = false;
  this->orientation = orientation;
  orientationValid = true;
  // orientationValid = false;
  d2Valid = false;

  return length;
}

double KVector2::getOrientation() const
{
  if (!orientationValid) {
    orientation = atan2(this->x, this->y);
    orientationValid = true;
  }

  return orientation;
}

void KVector2::rotate(double angleRadians)
{
  double currentLength = getD();
  orientation = addAngles(getOrientation(), angleRadians);

  this->x = currentLength * sin(orientation);
  this->y = currentLength * cos(orientation);
  orientationValid = true;
}

void KVector2::rotate(const KRotation* rotation)
{
  double newX = (this->x * rotation->cosTheta()) - (this->y * rotation->sinTheta());
  double newY = (this->x * rotation->sinTheta()) + (this->y * rotation->cosTheta());
  this->x = newX;
  this->y = newY;
  orientationValid = false;
}

void KVector2::printDebug() const
{
  // /*
  SerialUSB.print("(");
  SerialUSB.print(x, 10);
  SerialUSB.print(", ");
  SerialUSB.print(y, 10);
  SerialUSB.println(")");
  // */
}

double snapAngle(double angle)
{
  if (angle <= -M_PI)
    angle += 2.0d * M_PI;
  else if (angle > M_PI)
    angle -= 2.0d * M_PI;
  return angle;
}

double subtractAngles(double a1, double a2)
{
  return snapAngle(a1 - a2);
}

double addAngles(double a1, double a2)
{
  return snapAngle(a1 + a2);
}

double distanceBetween(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2.0d) + pow(y2 - y1, 2.0d));
}

double distanceBetween(const KVector2* v1, const KVector2* v2)
{
  return distanceBetween(v1->getX(), v1->getY(), v2->getX(), v2->getY());
}
