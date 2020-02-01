
#ifndef _KMATRIX2_H_
#define _KMATRIX2_H_

#include <Arduino.h>
#include "KVector2.h"
#include "KRotation2.h"

class KMatrix2
{

public:
  KVector2 position;
  KRotation2 orientation;

  KMatrix2()
  {}

  KMatrix2(const KMatrix2* m)
    : position(&m->position),
      orientation(&m->orientation)
  {}

  KMatrix2(const KVector2* v, double o)
    : position(v),
      orientation(o)
  {}

  KMatrix2(double x, double y, double o)
    : position(x, y),
      orientation(o)
  {}

  void set(const KMatrix2* m)
  {
      position.set(&m->position);
      orientation.set(&m->orientation);
  }

  void set(double x, double y, double o)
  {
      position.set(x, y);
      orientation.set(o);
  }

  void concat(const KMatrix2* m)
  {
      //unrotate and move
      position.unrotate(&m->orientation);
      position.addVector(&m->position);
      orientation.add(&m->orientation);
  }

  void unconcat(const KMatrix2* m)
  {
      //move and rotate
      orientation.subtract(&m->orientation);
      position.subtractVector(&m->position);
      position.rotate(&m->orientation);
  }

  void reset() {
    position.reset();
    orientation.reset();
  }

  void printDebug() const
  {
    SerialUSB.print("(");
    SerialUSB.print(position.getX(), 2);
    SerialUSB.print(", ");
    SerialUSB.print(position.getY(), 2);
    SerialUSB.print(", ");
    SerialUSB.print((180.0d * orientation.get()) / M_PI, 2);
    SerialUSB.println(")");
  }

};

void calculateRelativeBiArcKnot(KMatrix2* relativeTargetPosition);
void calculateRelativeBiArcKnot(const KMatrix2* relativeTargetPosition, KMatrix2* knot);

#endif
