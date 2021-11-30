
#include <SPI.h>
#include "zippies/math/ZVector2.h"

#define EPSILON 0.0001

double pad(double value, double epsilon)
{
    if (value == 0.0)
        return 0.0;

    return value < 0.0
        ? min(value, -epsilon)
        : max(value, epsilon);
}

ZVector2::ZVector2()
{
  reset();
}

ZVector2::ZVector2(const ZVector2* v)
  : x(v->x),
    y(v->y),
    d(v->d),
    dValid(v->dValid),
    d2(v->d2),
    d2Valid(v->d2Valid),
    _arctan(v->_arctan),
    arctanValid(v->arctanValid),
    _arctan2(v->_arctan2),
    arctan2Valid(v->arctan2Valid)
{
}

ZVector2::ZVector2(double x,
                   double y)
{
    this->x = x;
    this->y = y;
    d2Valid = false;
    dValid = false;
    arctanValid = false;
    arctan2Valid = false;
}

ZVector2::ZVector2(double x,
                   double y,
                   double ofLength)
{
  set(x, y, ofLength);
}

double ZVector2::getD() const
{
  if (!dValid) {
      d = sqrt(this->getD2());
      dValid = true;
  }
  return d;
}

double ZVector2::getD2() const
{
  if (!d2Valid) {
      if (dValid)
          d2 = d*d;
      else
          d2 = sq(x) + sq(y);
      d2Valid = true;
  }

  return d2;
}

bool ZVector2::equalsVector(const ZVector2* v) const
{
  return fabs(v->x-x) < EPSILON && fabs(v->y-y) < EPSILON;
}

/**
 * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
 * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
 * result will be just cos(ϴ).
 */
double ZVector2::dotVector(const ZVector2* v) const
{
  return (x*v->x) + (y*v->y);
}

double ZVector2::dotVector(double x2, double y2) const
{
  return (x*x2) + (y*y2);
}

double ZVector2::dotOrientation(double o) const
{
  return (x * sin(o)) + (y * cos(o));
}

double ZVector2::dotOrientation(const ZRotation2* orientation) const
{
  return (x * orientation->sin()) + (y * orientation->cos());
}

double ZVector2::crossProduct(const ZVector2* v) const
{
  return (x*v->y) - (y*v->x);
}

double ZVector2::crossProduct(const ZRotation2* o) const
{
  return (x*o->cos()) - (y*o->sin());
}

void ZVector2::multiply(double m)
{
  x *= m;
  y *= m;
  if (dValid)
    d *= m;
  d2Valid = false;
}

void ZVector2::reset()
{
  x = 0.0;
  y = 0.0;
  d = 0.0;
  dValid = true;
  d2 = 0.0;
  d2Valid = true;
  _arctan = 0.0;
  arctanValid = true;
  _arctan2 = 0.0;
  arctan2Valid = true;
}

void ZVector2::setX(double newX)
{
    if (x == newX)
        return;

    x = newX;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2::setY(double newY)
{
    if (y == newY)
        return;

    y = newY;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2::set(const ZVector2* v)
{
  this->set(v->x, v->y);
}

void ZVector2::set(double newX,
                   double newY)
{
    x = newX;
    y = newY;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2::set(double newX,
                   double newY,
                   double ofLength)
{
  if (ofLength == 0.0 || (newX == 0.0 && newY == 0.0)) {
    reset();
    return;
  }

  //calculate the actual values from the unit vector
  double lengthRatio = ofLength / getD();
  this->x *= lengthRatio;
  this->y *= lengthRatio;
  d = abs(ofLength);
  dValid = true;
  d2 = d*d;
  d2Valid = true;
  arctanValid = false;
  arctan2Valid = false;
}

// /*
void ZVector2::setD(double newD)
{
  // set(this->x, this->y, newD);
  double lengthRatio = newD / getD();
  this->x *= lengthRatio;
  this->y *= lengthRatio;
  d = abs(newD);
  dValid = true;
  d2 = d*d;
  d2Valid = true;
  arctanValid = false;
  arctan2Valid = false;
}
// */

void ZVector2::add(const ZVector2* v)
{
  add(v->x, v->y);
}

void ZVector2::add(double x, double y)
{
  this->x += x;
  this->y += y;
  dValid = false;
  d2Valid = false;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2::subtract(const ZVector2* v)
{
  subtract(v->x, v->y);
}

void ZVector2::subtract(double x, double y)
{
  this->x -= x;
  this->y -= y;
  dValid = false;
  d2Valid = false;
  arctanValid = false;
  arctan2Valid = false;
}

double ZVector2::projectAlong(double orientation)
{
  double sinTheta = sin(orientation);
  double cosTheta = cos(orientation);
  double dotProduct = (x * sinTheta) + (y * cosTheta);
  this->x = dotProduct * sinTheta;
  this->y = dotProduct * cosTheta;
  double length = dotProduct / cosTheta;
  this->d = abs(length);
  dValid = true;
  //the result could be either the arctan2 specified or +M_PI, exactly the opposite direction
  arctanValid = false;
  this->_arctan2 = dotProduct >= 0.0 ? orientation : addAngles(orientation, M_PI);
  arctan2Valid = true;
  d2Valid = false;

  return length;
}

double ZVector2::projectToward(double orientation)
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
  arctanValid = false;
  this->_arctan2 = orientation;
  arctan2Valid = true;
  d2Valid = false;

  return length;
}

double ZVector2::atan() const
{
  if (!arctanValid) {
    _arctan = atanSafe(this->x, this->y);
    arctanValid = true;
  }

  return _arctan;
}

double ZVector2::atan2() const
{
  if (!arctan2Valid) {
    _arctan2 = ::atan2(this->x, this->y);
    arctan2Valid = true;
  }

  return _arctan2;
}

/*
void ZVector2::rotate(double angleRadians)
{
  double currentLength = getD();
  this->_arctan2 = addAngles(atan2(), angleRadians);

  this->x = currentLength * sin(this->_arctan2);
  this->y = currentLength * cos(this->_arctan2);
  arctan2Valid = true;
  arctanValid = false;
}
*/

void ZVector2::rotate(const ZRotation2* rotation)
{
  double newX = (this->x *  rotation->cos()) + (this->y *  rotation->sin());
  double newY = (this->x * -rotation->sin()) + (this->y *  rotation->cos());
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2::rotate(double rotation)
{
  double newX = (this->x *  cos(rotation)) + (this->y *  sin(rotation));
  double newY = (this->x * -sin(rotation)) + (this->y *  cos(rotation));
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2::unrotate(const ZRotation2* rotation)
{
  double newX = (this->x *  rotation->cos()) + (this->y * -rotation->sin());
  double newY = (this->x *  rotation->sin()) + (this->y *  rotation->cos());
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2::unrotate(double rotation)
{
  double newX = (this->x *  cos(rotation)) + (this->y * -sin(rotation));
  double newY = (this->x *  sin(rotation)) + (this->y *  cos(rotation));
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2::printDebug() const
{
  // /*
  SerialUSB.print("(");
  SerialUSB.print(x, 10);
  SerialUSB.print(", ");
  SerialUSB.print(y, 10);
  SerialUSB.println(")");
  // */
}

double distanceBetween(double x1, double y1, double x2, double y2)
{
  return sqrt(sq(x2 - x1) + sq(y2 - y1));
}

double distanceBetween(const ZVector2* v1, const ZVector2* v2)
{
  return distanceBetween(v1->getX(), v1->getY(), v2->getX(), v2->getY());
}

double atanSafe(double x, double y)
{
    if (y == 0.0) {
      if (x == 0.0)
        return  0.0;
      return x < 0.0 ? -M_PI_2 : M_PI_2;
    }

    return ::atan(x / y);
}