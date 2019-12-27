
#include <Arduino.h>
#include <math.h>
#include "zippies/math/KVector3.h"

#define EPSILON 0.0001d

KVector3::KVector3()
: KVector3::KVector3(0, 0, 0)
{
}

KVector3::KVector3(KVector3* v)
  : KVector3::KVector3(v->x, v->y, v->z)
{
}

KVector3::KVector3(double x,
                   double y,
                   double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    d2 = 0.0d;
    d2Valid = false;
    d = 0.0d;
    dValid = false;
}

KVector3::KVector3(double x,
                   double y,
                   double z,
                   double ofLength)
{
  set(x, y, z, ofLength);
}

void KVector3::rotate(double w2, double x2, double y2, double z2)
{
  //old version of quaternion-vector multiplication; euqally accurate; more math computations
  //deprecated and will be deleted eventually
  /*
  double w1 =              - (x2*this->x) - (y2*this->y) - (z2*this->z);
  double x1 = (w2*this->x)                + (y2*this->z) - (z2*this->y);
  double y1 = (w2*this->y) - (x2*this->z)                + (z2*this->x);
  double z1 = (w2*this->z) + (x2*this->y) - (y2*this->x)               ;

  this->x =    (w1*(-x2))  + (x1*  w2 )  + (y1*(-z2))  - (z1*(-y2));
  this->y =    (w1*(-y2))  - (x1*(-z2))  + (y1*  w2 )  + (z1*(-x2));
  this->z =    (w1*(-z2))  + (x1*(-y2))  - (y1*(-x2))  + (z1*  w2 );
  */

  //credit goes to this source for describing a more efficient version of quaternion-vector multiplication
  //    https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication
  //
  // t = 2 * (q.xyz cross v)
  double tx = 2.0d * ((y2*this->z) - (z2*this->y));
  double ty = 2.0d * ((z2*this->x) - (x2*this->z));
  double tz = 2.0d * ((x2*this->y) - (y2*this->x));
  // v' = v + (q.w * t) + (q.xyz cross t)
  this->x += (w2 * tx) + ((y2*tz) - (z2*ty));
  this->y += (w2 * ty) + ((z2*tx) - (x2*tz));
  this->z += (w2 * tz) + ((x2*ty) - (y2*tx));
}

void KVector3::rotate(KQuaternion3* q)
{
  rotate(q->getW(), q->getX(), q->getY(), q->getZ());
}

void KVector3::unrotate(KQuaternion3* q)
{
  rotate(q->getW(), -q->getX(), -q->getY(), -q->getZ());
}

double KVector3::getD() {
    if (!dValid) {
        d = sqrt(this->getD2());
        dValid = true;
    }
    return d;
}

double KVector3::getD2() {
    if (!d2Valid) {
        if (dValid)
            d2 = d*d;
        else
            d2 = this->x*this->x + this->y*this->y + this->z*this->z;
        d2Valid = true;
    }

    return d2;
}

bool KVector3::equalsVector(KVector3* v) {
    return (fabs(v->x-x) < EPSILON && fabs(v->y-y) < EPSILON && fabs(v->z-z) < EPSILON);
}

double KVector3::dotVector(KVector3* v) {
    return (x*v->x) + (y*v->y) + (z*v->z);
}

void KVector3::crossVector(KVector3* v) {
  double newX = (this->y * v->z) - (this->z * v->y);
  double newY = (this->z * v->x) - (this->x * v->z);
  double newZ = (this->x * v->y) - (this->y * v->x);
  this->x = newX;
  this->y = newY;
  this->z = newZ;
  dValid = false;
  d2Valid = false;
}

double KVector3::angleToVector(KVector3* v) {
  return acos( dotVector(v) / (this->getD() * v->getD()) );
}

void intersectPlane(KVector3* planeNormal,
                    KVector3* fromVector,
                    KVector3* throughVector,
                    KVector3* result)
{
  double t = -planeNormal->dotVector(fromVector)/
          planeNormal->dotVector(throughVector);
  result->set(fromVector->getX() + (throughVector->getX() * t),
              fromVector->getY() + (throughVector->getY() * t),
              fromVector->getZ() + (throughVector->getZ() * t));
}

void KVector3::setX(double newX) {
    if (x == newX)
        return;

    x = newX;
    dValid = false;
    d2Valid = false;
}

void KVector3::setY(double newY) {
    if (y == newY)
        return;

    y = newY;
    dValid = false;
    d2Valid = false;
}

void KVector3::setZ(double newZ) {
    if (z == newZ)
        return;

    z = newZ;
    dValid = false;
    d2Valid = false;
}

void KVector3::set(double newX,
                   double newY,
                   double newZ)
{
    x = newX;
    y = newY;
    z = newZ;
    dValid = false;
    d2Valid = false;
}

void KVector3::set(double newX,
                   double newY,
                   double newZ,
                   double ofLength)
{
  if (ofLength == 0.0d || (newX == 0.0d && newY == 0.0d && newZ == 0.0d)) {
    this->x = 0.0d;
    this->y = 0.0d;
    this->z = 0.0d;
    d = 0.0d;
    dValid = true;
    d2 = 0.0d;
    d2Valid = true;
    return;
  }

  //calculate the actual values from the unit vector
  double vd = sqrt(newX*newX+newY*newY+newZ*newZ);
  this->x = (newX*ofLength)/vd;
  this->y = (newY*ofLength)/vd;
  this->z = (newZ*ofLength)/vd;
  d = abs(ofLength);
  dValid = true;
  d2 = d*d;
  d2Valid = true;
}

void KVector3::setD(double newD)
{
  double previousD = getD();
  if (previousD == 0.0d || newD == 0.0d) {
    this->x = 0.0d;
    this->y = 0.0d;
    this->z = 0.0d;
    d = 0.0d;
    dValid = true;
    d2 = 0.0d;
    d2Valid = true;
    return;
  }

  double ratio = previousD / newD;
  this->x *= ratio;
  this->y *= ratio;
  this->z *= ratio;
  d = abs(newD);
  dValid = true;
  d2 = d*d;
  d2Valid = true;
}

void KVector3::printDebug()
{
  SerialUSB.print(x, 10);
  SerialUSB.print("  ");
  SerialUSB.print(y, 10);
  SerialUSB.print("  ");
  SerialUSB.println(z, 10);
}
