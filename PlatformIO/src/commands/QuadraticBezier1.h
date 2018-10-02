
#ifndef _QUADRATICBEZIER_H_
#define _QUADRATICBEZIER_H_

double quadraticLength(double ax, double ay,
                       double bx, double by,
                       double cx, double cy);

double quadraticLerp0(double t,
                      double c1,
                      double a2);

double quadraticLerp(double t,
                     double a1,
                     double c1,
                     double a2);

class QuadraticBezier1
{

private:
  double controlPoint;
  double endPoint;

public:
  QuadraticBezier1()
    : controlPoint(0.0d),
      endPoint(0.0d)
  {}

  void set(double ep) {
    set(ep / 2.0d, ep);
  }

  void set(double cp, double ep) {
    controlPoint = cp;
    endPoint = ep;
  }

  double lerp(double atNormalizedTime) {
    return quadraticLerp0(atNormalizedTime, controlPoint, endPoint);
  }

};

#endif
