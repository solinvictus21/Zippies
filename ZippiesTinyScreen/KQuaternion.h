
#pragma once
#define EPSILON 0.0001d

class KQuaternion
{

private:
  double x;
  double y;
  double z;
  double w;

  void multiplyByW(double ax, double ay, double az, double w);
  void setWithW(double x, double y, double z, double w);
  void normalize();

public:
  KQuaternion();
  KQuaternion(double x, double y, double z, double angle);

  double getX() { return x; }
  double getY() { return y; }
  double getZ() { return z; }
  double getW() { return w; }
  void set(double x, double y, double z, double angle);

  void rotateX(double angle);
  void rotateY(double angle);
  void rotateZ(double angle);
  void rotate(double x, double y, double z, double angle);
  
};

