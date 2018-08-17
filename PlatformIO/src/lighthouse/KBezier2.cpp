
#include <SPI.h>
#include <math.h>
#include "KBezier2.h"

KBezier2::KBezier2()
  : totalLength(0)
{
}

KBezier2::KBezier2(KVector2* cp, KVector2* ep)
  : controlPoint(cp),
    endPoint(cp)
{
  recalculateLength();
}

void KBezier2::lerpPoint(double atDistance, KVector2* lerpedPoint)
{
  double ex = endPoint.getX();
  double ey = endPoint.getY();
  double normalizedTime = atDistance / totalLength;
  // lerpedPoint->set(lerp(0.0d, ex * (1.0d - BEZIER_INTERPOLATION), ex, ex * BEZIER_INTERPOLATION, normalizedTime),
      // lerp(0.0d, ey * (1.0d - BEZIER_INTERPOLATION), ey, ey * BEZIER_INTERPOLATION, normalizedTime));
  lerpedPoint->set(lerp(0.0d, controlPoint.getX(), ex, normalizedTime),
      lerp(0.0d, controlPoint.getY(), ey, normalizedTime));
  // endPoint.printDebug();
}

void KBezier2::set(double cx, double cy, double ex, double ey)
{
  controlPoint.set(cx, cy);
  endPoint.set(ex, ey);

  recalculateLength();
}

void KBezier2::recalculateLength()
{
  //recalculate bezier length
  // v.x = 2*(b.x - a.x);
  double vx = 2.0d * controlPoint.getX();
  // v.y = 2*(b.y - a.y);
  double vy = 2.0d * controlPoint.getY();
  // w.x = c.x - 2*b.x + a.x;
  double wx = endPoint.getX() - (2.0d * controlPoint.getX());
  // w.y = c.y - 2*b.y + a.y;
  double wy = endPoint.getY() - (2.0d * controlPoint.getY());

  // uu = 4*(w.x*w.x + w.y*w.y);
  double uu = 4.0d * ((wx * wx) + (wy * wy));
  if (uu < 0.0001) {
      totalLength = endPoint.getD();
      return;
  }

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

  totalLength = (double)((t1 * t2 - t3 * log(t2 + t1) - (vv * t4 - t3 * log(vv + t4))) / (8.0d * pow(uu, 1.5d)));
  /*
  SerialUSB.println("data: ");
  controlPoint.printDebug();
  endPoint.printDebug();
  SerialUSB.println(vx, 2);
  SerialUSB.println(vy, 2);
  SerialUSB.println(wx, 2);
  SerialUSB.println(wy, 2);
  SerialUSB.println(uu, 2);
  SerialUSB.println(vv, 2);
  SerialUSB.println(t4, 2);
  SerialUSB.println(totalLength, 2);
  SerialUSB.println();
  */
}

double lerp(double a1,
            double c1,
            double a2,
            double t)
{
  double a1c1 = a1+((c1-a1)*t);
  double c1a2 = c1+((a2-c1)*t);
  return a1c1+((c1a2-a1c1)*t);
}

double lerp(double a1,
            double c1,
            double a2,
            double c2,
            double t)
{
  double a1c1 = a1+((c1-a1)*t);
  double c1c2 = c1+((c2-c1)*t);
  double c2a2 = c2+((a2-c2)*t);
  double a1c1c1c2 = a1c1+((c1c2-a1c1)*t);
  double c1c2c2a2 = c1c2+((c2a2-c1c2)*t);
  return a1c1c1c2+((c1c2c2a2-a1c1c1c2)*t);
}
