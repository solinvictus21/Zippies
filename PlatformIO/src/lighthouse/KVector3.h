
#ifndef _KVECTOR3_H_
#define _KVECTOR3_H_

#include "KQuaternion3.h"

class KVector3
{
protected:
  double d, d2;
  bool dValid, d2Valid;
  double x;
  double y;
  double z;

  void setD(double newD, KVector3* unitVector);
  void vectorChanged();

public:
  KVector3();
  KVector3(KVector3* v);
  KVector3(double x,
           double y,
           double z);
  KVector3(double x,
           double y,
           double z,
           double ofLength);

  void setX(double x);
  double getX() { return x; }
  void setY(double y);
  double getY() { return y; }
  void setZ(double z);
  double getZ() { return z; }
  void set(double x, double y, double z);
  void set(double x, double y, double z, double ofLength);

  double getD();
  double getD2();
  void setD(double newD);
  void normalize() { this->setD(1.0f); }

  bool equalsVector(KVector3* v);
  double dotVector(KVector3* v);
  void crossVector(KVector3* v);
  double angleToVector(KVector3* v);

  void rotate(KQuaternion3* q);
  void unrotate(KQuaternion3* q);
  void rotate(double x2, double y2, double z2, double w2);

  void printDebug();

};

void intersectPlane(KVector3* planeNormal,
                    KVector3* fromVector,
                    KVector3* throughVector,
                    KVector3* result);

#endif
