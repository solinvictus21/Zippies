
#ifndef _ZVECTOR2OM_H_
#define _ZVECTOR2OM_H_

#include "ZRotation2.h"

class ZVector2OM
{

protected:
    double o;
    double m;
    mutable double d2;
    mutable bool d2Valid = false;
    mutable double _arctan;
    mutable bool arctanValid = false;

public:
    ZVector2OM();
    ZVector2OM(const ZVector2OM* v);
    ZVector2OM(double o, double m);

    void reset();
    /*
    void setO(double x);
    double getO() const { return o; }
    void setM(double y);
    double getM() const { return m; }
    void set(const ZVector2OM* v);
    void set(double o, double m);
    void rotate(const ZRotation2* rotation);
    void rotate(double rotation);
    void unrotate(const ZRotation2* rotation);
    void unrotate(double rotation);
    void flip();

    double getD() const;
    void setD(double newD);
    double getD2() const;
    double atan() const;
    double atan2() const;
    void normalize() { this->set(this->x, this->y, 1.0); }

    bool equalsVector(const ZVector2OM* v) const;
    double dotVector(double x2, double y2) const;
    double dotVector(const ZVector2OM* v) const;
    double dotOrientation(double orientation) const;
    double dotOrientation(const ZRotation2* orientation) const;
    double crossProduct(const ZVector2OM* v) const;
    double crossProduct(const ZRotation2* v) const;

    void add(const ZVector2OM* v);
    void add(double x, double y);
    void subtract(const ZVector2OM* v);
    void subtract(double x, double y);
    void multiply(double factor);
    double projectAlong(double orientation);
    double projectToward(double orientation);
    double angleToVector(const ZVector2OM* v) const { return angleToOrientation(v->atan2()); }
    double angleToOrientation(double a) const { return subtractAngles(a, atan2()); }

    void printDebug() const;
    */

};

#endif
