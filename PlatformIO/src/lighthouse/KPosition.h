
#ifndef _KPOSITION_H_
#define _KPOSITION_H_

#include "KVector2.h"

class KPosition
{

public:
  KPosition()
  {}

  KPosition(const KPosition* p)
    : KPosition(&p->vector, p->orientation)
  {}

  KPosition(const KVector2* p, double o)
    : vector(p),
      orientation(o)
  {}

  KPosition(double x, double y, double o)
    : vector(x, y),
      orientation(o)
  {}

  void set(const KPosition* p)
  {
    vector.set(&p->vector);
    orientation = p->orientation;
  }

  void set(double x, double y, double o)
  {
    vector.set(x, y);
    orientation = o;
  }

  KVector2 vector;
  double orientation;

};

#endif
