
#include <SPI.h>
#include "zippies/math/ZVector2OM.h"

#define EPSILON 0.0001

ZVector2OM::ZVector2OM()
{
  reset();
}

ZVector2OM::ZVector2OM(const ZVector2OM* v)
  : o(v->o),
    m(v->m),
    d2(v->d2),
    d2Valid(v->d2Valid)
{
}

ZVector2OM::ZVector2OM(double o, double m)
{
    this->o = o;
    this->m = m;
}

void ZVector2OM::reset()
{
    o = 0.0;
    m = 0.0;
    d2 = 0.0;
    d2Valid = true;
    _arctan = 0.0;
    arctanValid = true;
}

/*
ZVector2OM::ZVector2OM(double o, double m)
{
    set(o, m);
}

double ZVector2OM::getD() const
{
    return abs(m);
}

double ZVector2OM::getD2() const
{
    return sq(m);
}

bool ZVector2OM::equalsVector(const ZVector2OM* v) const
{
  return fabs(v->x-x) < EPSILON && fabs(v->y-y) < EPSILON;
}
*/

/**
 * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
 * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
 * result will be just cos(ϴ).
 */
/*
double ZVector2OM::dotVector(const ZVector2OM* v) const
{
  return (x*v->x) + (y*v->y);
}

double ZVector2OM::dotVector(double x2, double y2) const
{
  return (x*x2) + (y*y2);
}

double ZVector2OM::dotOrientation(double o) const
{
  return (x * sin(o)) + (y * cos(o));
}

double ZVector2OM::dotOrientation(const ZRotation2* orientation) const
{
  return (x * orientation->sin()) + (y * orientation->cos());
}

double ZVector2OM::crossProduct(const ZVector2OM* v) const
{
  return (x*v->y) - (y*v->x);
}

double ZVector2OM::crossProduct(const ZRotation2* o) const
{
  return (x*o->cos()) - (y*o->sin());
}

void ZVector2OM::multiply(double m)
{
  x *= m;
  y *= m;
  if (dValid)
    d *= m;
  d2Valid = false;
}

void ZVector2OM::reset()
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

void ZVector2OM::setX(double newX)
{
    if (x == newX)
        return;

    x = newX;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2OM::setY(double newY)
{
    if (y == newY)
        return;

    y = newY;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2OM::set(const ZVector2OM* v)
{
  this->set(v->x, v->y);
}

void ZVector2OM::set(double newX,
                   double newY)
{
    x = newX;
    y = newY;
    dValid = false;
    d2Valid = false;
    arctanValid = false;
    arctan2Valid = false;
}

void ZVector2OM::set(double newX,
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

void ZVector2OM::add(const ZVector2OM* v)
{
  add(v->x, v->y);
}

void ZVector2OM::add(double x, double y)
{
  this->x += x;
  this->y += y;
  dValid = false;
  d2Valid = false;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2OM::subtract(const ZVector2OM* v)
{
  subtract(v->x, v->y);
}

void ZVector2OM::subtract(double x, double y)
{
  this->x -= x;
  this->y -= y;
  dValid = false;
  d2Valid = false;
  arctanValid = false;
  arctan2Valid = false;
}

double ZVector2OM::projectAlong(double orientation)
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

double ZVector2OM::projectToward(double orientation)
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

double ZVector2OM::atan() const
{
  if (!arctanValid) {
    _arctan = atanSafe(this->x, this->y);
    arctanValid = true;
  }

  return _arctan;
}

double ZVector2OM::atan2() const
{
  if (!arctan2Valid) {
    _arctan2 = ::atan2(this->x, this->y);
    arctan2Valid = true;
  }

  return _arctan2;
}

void ZVector2OM::rotate(const ZRotation2* rotation)
{
  double newX = (this->x *  rotation->cos()) + (this->y *  rotation->sin());
  double newY = (this->x * -rotation->sin()) + (this->y *  rotation->cos());
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2OM::rotate(double rotation)
{
  double newX = (this->x *  cos(rotation)) + (this->y *  sin(rotation));
  double newY = (this->x * -sin(rotation)) + (this->y *  cos(rotation));
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2OM::unrotate(const ZRotation2* rotation)
{
  double newX = (this->x *  rotation->cos()) + (this->y * -rotation->sin());
  double newY = (this->x *  rotation->sin()) + (this->y *  rotation->cos());
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2OM::unrotate(double rotation)
{
  double newX = (this->x *  cos(rotation)) + (this->y * -sin(rotation));
  double newY = (this->x *  sin(rotation)) + (this->y *  cos(rotation));
  this->x = newX;
  this->y = newY;
  arctanValid = false;
  arctan2Valid = false;
}

void ZVector2OM::flip() {
  x = -x;
  y = -y;
  arctan2Valid = false;
}

void ZVector2OM::printDebug() const
{
  SerialUSB.print("(");
  SerialUSB.print(x, 5);
  SerialUSB.print(", ");
  SerialUSB.print(y, 5);
  SerialUSB.println(")");
}
*/