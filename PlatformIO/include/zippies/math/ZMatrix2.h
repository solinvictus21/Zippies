
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

    /**
     * @brief Append the designated position and orientation to this position and orientation.
     * 
     * @param m the designated position and orientation to append
     */
    void concat(const ZMatrix2* m)
    {
        position.add(
            (m->position.getX() * this->orientation.cos()) + (m->position.getY() * this->orientation.sin()),
            (m->position.getX() * -this->orientation.sin()) + (m->position.getY() * this->orientation.cos()));
        orientation.add(&m->orientation);
    }

    /**
     * @brief Append this position and orientation to the designated position and orientation.
     * 
     * @param m the designated position and orientation to be appended to
     */
    void concatTo(const ZMatrix2* m)
    {
        //unrotate and move
        position.rotate(&m->orientation);
        position.add(&m->position);
        orientation.add(&m->orientation);
    }

    /**
     * @brief Calculate the relative position and orientation from the designed orientation and position to the
     *        current position and orientation.
     * 
     * @param m the designation position and orientation from which to calculate the relative position and orientation
     */
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
        SerialUSB.print(position.getX(), 5);
        SerialUSB.print(", ");
        SerialUSB.print(position.getY(), 5);
        SerialUSB.print(", ");
        SerialUSB.print((180.0 * orientation.get()) / M_PI, 5);
        SerialUSB.println(")");
    }

};

void calculateRelativeBiArcKnot(ZMatrix2* relativeTargetPosition);
void calculateRelativeBiArcKnot(const ZMatrix2* relativeTargetPosition, ZMatrix2* knot);

#endif
