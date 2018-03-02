
#pragma once
#include "KQuaternion.h"

class KVector2
{
private:
  double x;
  double y;
  double d, d2;
  bool dValid, d2Valid;
  double orientation;
  bool orientationValid;

  void setD(double newD, KVector2* unitVector);

public:
  KVector2();
  KVector2(KVector2* v);
  KVector2(double x,
           double y);
  KVector2(double x,
           double y,
           double ofLength);

  void reset();
  void setX(double x);
  double getX() { return x; }
  void setY(double y);
  double getY() { return y; }
  void set(KVector2* v);
  void set(double x, double y);
  void set(double x, double y, double ofLength);
  void add(KVector2* v);
  void rotate(double angleRadians);

  double getD();
  double getD2();
  double getOrientation();
  void setD(double newD);
  void normalize() { this->setD(1.0f); }
  
  bool equalsVector(KVector2* v);
  double dotVector(KVector2* v);
  double angleToVector(KVector2* v);

  void printDebug();

};

class KVector3
{
private:
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

  void rotate(KQuaternion* q);
  void unrotate(KQuaternion* q);
  void rotate(double x2, double y2, double z2, double w2);

  void printDebug();

};


class KCubicBezier
{
private:
    KVector2* anchor1;
    KVector2* control1;
    KVector2* anchor2;
    KVector2* control2;
    
    double ax, bx, cx;
    double ay, by, cy;
    double az, bz, cz;

public:
  KCubicBezier(KVector2*a1, KVector2* c1, KVector2* a2, KVector2* c2);

/*
+(GLfloat)lerpFromA1:(GLfloat)a1
                  c1:(GLfloat)c1
                  a2:(GLfloat)a2
                  c2:(GLfloat)c2
                   t:(GLfloat)t;
*/

};

void intersectPlane(KVector3* planeNormal,
                    KVector3* fromVector,
                    KVector3* throughVector,
                    KVector3* result);

