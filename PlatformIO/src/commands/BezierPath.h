
#ifndef _BEZIERPATH_H_
#define _BEZIERPATH_H_

#include "../lighthouse/KPath.h"
#include "../lighthouse/KVector2.h"

class BezierPath : public KPath
{

private:
  const KVector2* startPoint;
  const KVector2* controlPoint;
  const KVector2* endPoint;
  double length;

public:
  BezierPath(const KVector2* sp, const KVector2* cp, const KVector2* ep)
    : startPoint(sp),
      controlPoint(cp),
      endPoint(ep)
  {
    //recalculate bezier length
    // v.x = 2*(b.x - a.x);
    double vx = 2.0d * (controlPoint->getX() - startPoint->getX());
    // v.y = 2*(b.y - a.y);
    double vy = 2.0d * (controlPoint->getY() - startPoint->getY());
    // w.x = c.x - 2*b.x + a.x;
    double wx = endPoint->getX() - (2.0d * controlPoint->getX()) + startPoint->getX();
    // w.y = c.y - 2*b.y + a.y;
    double wy = endPoint->getY() - (2.0d * controlPoint->getY()) + startPoint->getY();

    // uu = 4*(w.x*w.x + w.y*w.y);
    double uu = 4.0d * ((wx * wx) + (wy * wy));
    if (uu < 0.0001) {
        length = endPoint->getD();
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

    length = (double)((t1 * t2 - t3 * log(t2 + t1) - (vv * t4 - t3 * log(vv + t4))) / (8.0d * pow(uu, 1.5d)));
  }

  void lerp(double atNormalizedTime, KVector2* lerpedPoint) const {
    lerpedPoint->set(lerp3(startPoint->getX(), controlPoint->getX(), endPoint->getX(), atNormalizedTime),
        lerp3(startPoint->getY(), controlPoint->getY(), endPoint->getY(), atNormalizedTime));
  }

  double getLength() const { return length; }

};

#endif
