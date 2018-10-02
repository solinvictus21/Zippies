
#include <Arduino.h>
#include "QuadraticBezier1.h"

double quadraticLength(double ax, double ay,
                       double bx, double by,
                       double cx, double cy)
{
  //recalculate bezier length
  // w.x = c.x - 2*b.x + a.x;
  double wx = cx - (2.0d * bx) + ax;
  // w.y = c.y - 2*b.y + a.y;
  double wy = cy - (2.0d * by) + ay;

  // uu = 4*(w.x*w.x + w.y*w.y);
  double uu = 4.0d * ((wx * wx) + (wy * wy));
  if (uu < 0.0001)
      return sqrt(pow(cx-ax, 2.0d) + pow(cy-ay, 2.0d));

  // v.x = 2*(b.x - a.x);
  double vx = 2.0d * (bx - ax);
  // v.y = 2*(b.y - a.y);
  double vy = 2.0d * (by - ay);
  // vv = 4*(v.x*w.x + v.y*w.y);
  double vv = 4.0d * ((vx * wx) + (vy * wy));
  // ww = v.x*v.x + v.y*v.y;
  double ww = (vx * vx) + (vy * vy);

  // t1 = (float) (2*Math.sqrt(uu*(uu + vv + ww)));
  double t1 = 2.0d * sqrt(uu * (uu + vv + ww));
  // t2 = 2*uu+vv;
  double t2 = (2.0d * uu) + vv;
  // t3 = vv*vv - 4*uu*ww;
  double t3 = (vv * vv) - (4.0d * uu * ww);
  // t4 = (float) (2*Math.sqrt(uu*ww));
  double t4 = 2.0d * sqrt(uu * ww);

  return (t1 * t2 - t3 * log(t2 + t1) - (vv * t4 - t3 * log(vv + t4))) / (8.0d * pow(uu, 1.5d));
}

double quadraticLerp(double t,
                     double a1,
                     double c1,
                     double a2)
{
  double a1c1 = a1+((c1-a1)*t);
  double c1a2 = c1+((a2-c1)*t);
  return a1c1+((c1a2-a1c1)*t);
}

double quadraticLerp0(double t,
                      double c1,
                      double a2)
{
  double a1c1 = c1*t;
  double c1a2 = c1+((a2-c1)*t);
  return a1c1+((c1a2-a1c1)*t);
}
