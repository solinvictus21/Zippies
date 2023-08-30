
#ifndef _KVECTOR3_H_
#define _KVECTOR3_H_

#include "ZQuaternion3.h"

class ZVector3
{
protected:
  double d, d2;
  bool dValid, d2Valid;
  double x;
  double y;
  double z;

  void setD(double newD, ZVector3* unitVector);

public:
  ZVector3();
  ZVector3(ZVector3* v);
  ZVector3(double x,
           double y,
           double z);
  ZVector3(double x,
           double y,
           double z,
           double ofLength);

  void setX(double x);
  double getX() const { return x; }
  void setY(double y);
  double getY() const { return y; }
  void setZ(double z);
  double getZ() const { return z; }
  void set(double x, double y, double z);
  void set(double x, double y, double z, double ofLength);

  double getD();
  double getD2();
  void setD(double newD);
  void normalize() { this->setD(1.0f); }

  bool equalsVector(ZVector3* v);
  double dotVector(ZVector3* v);
  void crossVector(ZVector3* v);
  double angleToVector(ZVector3* v);

  void rotate(ZQuaternion3* q);
  void unrotate(ZQuaternion3* q);
  void rotate(double x2, double y2, double z2, double w2);

  void printDebug();

};

void intersectPlane(ZVector3* planeNormal,
                    ZVector3* fromVector,
                    ZVector3* throughVector,
                    ZVector3* result);

#endif
