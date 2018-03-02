
#include <Tinyscreen.h>
#include <math.h>
#include "KQuaternion.h"

KQuaternion::KQuaternion()
{
  x = y = z = 0.0d;
  w = 1.0d;
}

KQuaternion::KQuaternion(double x, double y, double z, double angle)
{
  set(x, y, z, angle);
}

void KQuaternion::set(double x, double y, double z, double angle)
{
  double sina2 = sin(angle / 2.0d);
  this->x = x * sina2;
  this->y = y * sina2;
  this->z = z * sina2;
  w = cos(angle / 2.0d);
  normalize();
}

void KQuaternion::rotateX(double angle)
{
  rotate(1.0d, 0.0d, 0.0d, angle);
}

void KQuaternion::rotateY(double angle)
{
  rotate(0.0d, 1.0d, 0.0d, angle);
}

void KQuaternion::rotateZ(double angle)
{
  rotate(0.0d, 0.0d, 1.0d, angle);
}

void KQuaternion::rotate(double ax,
                         double ay,
                         double az,
                         double angle)
{
  double sinfa2 = sin(angle/2.0d);
  multiplyByW(ax * sinfa2, ay * sinfa2, az * sinfa2, cos(angle/2.0d));
}

void KQuaternion::multiplyByW(double x2,
                              double y2,
                              double z2,
                              double w2)
{
//  SerialUSB.print(x2, 10);
//  SerialUSB.print(" ");
//  SerialUSB.print(y2, 10);
//  SerialUSB.print(" ");
//  SerialUSB.print(z2, 10);
//  SerialUSB.print(" ");
//  SerialUSB.println(w2, 10);
  setWithW((w*x2) + (x*w2) + (y*z2) - (z*y2),
           (w*y2) - (x*z2) + (y*w2) + (z*x2),
           (w*z2) + (x*y2) - (y*x2) + (z*w2),
           (w*w2) - (x*x2) - (y*y2) - (z*z2));
}

void KQuaternion::setWithW(double x,
                           double y,
                           double z,
                           double w)
{
//  SerialUSB.print(x, 10);
//  SerialUSB.print(" ");
//  SerialUSB.print(y, 10);
//  SerialUSB.print(" ");
//  SerialUSB.print(z, 10);
//  SerialUSB.print(" ");
//  SerialUSB.println(w, 10);
  this->x = x;
  this->y = y;
  this->z = z;
  this->w = w;
  
  normalize();
}

void KQuaternion::normalize()
{
    double magnitude2 = (w*w) + (x*x) + (y*y) + (z*z);
    if (fabs(magnitude2-1.0d) < EPSILON)
        return;
    
    double magnitude = sqrt(magnitude2);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
    w /= magnitude;
}

