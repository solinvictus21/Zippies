
#include <math.h>
#include "KVector2.h"

#define EPSILON 0.0001d

KVector2::KVector2()
{
  reset();
}

KVector2::KVector2(KVector2* v)
  : KVector2::KVector2(v->x, v->y)
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

double KVector2::getD()
{
    if (!dValid) {
        d = sqrt(this->getD2());
        dValid = true;
    }
    return d;
}

double KVector2::getD2()
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

bool KVector2::equalsVector(KVector2* v)
{
    return fabs(v->x-x) < EPSILON && fabs(v->y-y) < EPSILON;
}

/**
 * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
 * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
 * result will be just cos(ϴ).
 */
double KVector2::dotVector(KVector2* v)
{
    return (x*v->x) + (y*v->y);
}

double KVector2::angleToVector(KVector2* v)
{
  double angle = v->getOrientation() - getOrientation();
  if (angle < -M_PI)
    angle += 2 * M_PI;
  else if (angle > M_PI)
    angle -= 2 * M_PI;
  return angle;
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

void KVector2::set(KVector2* v)
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
}

void KVector2::addVector(KVector2* v)
{
  this->x += v->x;
  this->y += v->y;
  dValid = false;
  d2Valid = false;
  orientationValid = false;
}

void KVector2::subtractVector(KVector2* v)
{
  this->x -= v->x;
  this->y -= v->y;
  dValid = false;
  d2Valid = false;
  orientationValid = false;
}

double KVector2::getOrientation()
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
  orientation = getOrientation() + angleRadians;
  if (orientation < -M_PI)
    orientation += 2.0d * M_PI;
  else if (orientation > M_PI)
    orientation -= 2.0d * M_PI;

  this->x = currentLength * sin(orientation);
  this->y = currentLength * cos(orientation);
  orientationValid = true;
}

void KVector2::printDebug()
{
  /*
  SerialUSB.print("(");
  SerialUSB.print(x, 10);
  SerialUSB.print(", ");
  SerialUSB.print(y, 10);
  SerialUSB.println(")");
  */
}
