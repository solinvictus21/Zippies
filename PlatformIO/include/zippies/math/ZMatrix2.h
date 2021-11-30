
#ifndef _KMATRIX2_H_
#define _KMATRIX2_H_

#include <Arduino.h>
#include "ZVector2.h"
#include "ZRotation2.h"

class ZMatrix2
{

public:
    ZVector2 position;
    ZRotation2 orientation;

    ZMatrix2()
    {}

    ZMatrix2(const ZMatrix2* m)
      : position(&m->position),
        orientation(&m->orientation)
    {}

    ZMatrix2(const ZVector2* v, double o)
      : position(v),
        orientation(o)
    {}

    ZMatrix2(double x, double y, double o)
      : position(x, y),
        orientation(o)
    {}

    void set(const ZMatrix2* m)
    {
        position.set(&m->position);
        orientation.set(&m->orientation);
    }

    void set(double x, double y, double o)
    {
        position.set(x, y);
        orientation.set(o);
    }

    void concat(const ZVector2* v)
    {
        position.add(
            (v->getX() * this->orientation.cos()) + (v->getY() * this->orientation.sin()),
            (v->getX() * -this->orientation.sin()) + (v->getY() * this->orientation.cos()));
        orientation.add(2.0 * v->atan());
    }

    void concat(const ZMatrix2* m)
    {
        position.add(
            (m->position.getX() * this->orientation.cos()) + (m->position.getY() * this->orientation.sin()),
            (m->position.getX() * -this->orientation.sin()) + (m->position.getY() * this->orientation.cos()));
        orientation.add(&m->orientation);
    }

    void concatTo(const ZMatrix2* m)
    {
        //unrotate and move
        position.rotate(&m->orientation);
        position.add(&m->position);
        orientation.add(&m->orientation);
    }

    void unconcatFrom(const ZMatrix2* m)
    {
        //move and rotate
        orientation.subtract(&m->orientation);
        position.subtract(&m->position);
        position.unrotate(&m->orientation);
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
        SerialUSB.print((180.0 * orientation.get()) / M_PI, 2);
        SerialUSB.println(")");
    }

};

void calculateRelativeBiArcKnot(ZMatrix2* relativeTargetPosition);
void calculateRelativeBiArcKnot(const ZMatrix2* relativeTargetPosition, ZMatrix2* knot);

#endif
