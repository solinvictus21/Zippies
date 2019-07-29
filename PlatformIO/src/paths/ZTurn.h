
#ifndef _ZTURN_H_
#define _ZTURN_H_

class ZTurn
{

private:
  double startOrientation;
  double deltaOrientation;
  bool inReverse;

public:
  ZTurn(double so, double do)
    : startOrientation(so),
      deltaOrientation(do),
      inReverse(false)
  {}

  ZTurn(double so, double do, bool r)
    : startOrientation(so),
      deltaOrientation(do),
      inReverse(r)
  {}

  double interpolate(double interpolatedTime) {
    return startOrientation + (deltaOrientation * interpolatedTime);
  }

};

#endif
