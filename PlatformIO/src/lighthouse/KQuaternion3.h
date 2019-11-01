
#ifndef _KQUATERNION3_H_
#define _KQUATERNION3_H_

class KQuaternion3
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
  KQuaternion3();
  KQuaternion3(double x, double y, double z, double angle);

  double getX() { return x; }
  double getY() { return y; }
  double getZ() { return z; }
  double getW() { return w; }
  void set(double x, double y, double z, double angle);

  void rotateX(double angle);
  void rotateY(double angle);
  void rotateZ(double angle);
  void rotate(double x, double y, double z, double angle);

  void printDebug();

};

#endif
