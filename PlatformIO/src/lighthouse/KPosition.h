
#ifndef _KPOSITION_H_
#define _KPOSITION_H_

#include "KVector2.h"

typedef struct _KPosition
{

  KVector2 vector;
  double orientation;

} KPosition;

double distanceBetween(const KVector2* v1, double o1, const KVector2 v2, double o2);

#endif
