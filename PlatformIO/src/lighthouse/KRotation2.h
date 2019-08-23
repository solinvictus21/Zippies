
#ifndef _KROTATION2_H_
#define _KROTATION2_H_

#include <Arduino.h>
extern double subtractAngles(double a1, double a2);
extern double addAngles(double a1, double a2);
extern double snapAngle(double angle);

class KRotation2
{

private:
  double _theta;
  mutable double cosT;
  mutable bool cosTValid = false;
  mutable double sinT;
  mutable bool sinTValid = false;

public:
  KRotation2()
    : _theta(0.0d)
  {}

  KRotation2(const KRotation2* r)
    : _theta(r->_theta),
      cosT(r->cosT),
      cosTValid(r->cosTValid),
      sinT(r->sinT),
      sinTValid(r->sinTValid)
  {}

  KRotation2(double t)
    : _theta(t)
  {}

  double theta() const {
    return this->_theta;
  }

  void set(const KRotation2* r) {
    this->_theta = r->_theta;
    this->cosT = r->cosT;
    this->cosTValid = r->cosTValid;
    this->sinT = r->sinT;
    this->sinTValid = r->sinTValid;
  }

  void set(double t) {
    this->_theta = t;
    cosTValid = false;
    sinTValid = false;
  }

  double get() const {
    return this->_theta;
  }

  void add(double o) {
    this->_theta = addAngles(this->_theta, o);
    cosTValid = false;
    sinTValid = false;
  }

  void add(const KRotation2* r) {
    this->add(r->_theta);
  }

  void subtract(double o) {
    this->_theta = subtractAngles(this->_theta, o);
    cosTValid = false;
    sinTValid = false;
  }

  void subtract(const KRotation2* r) {
    this->subtract(r->_theta);
  }

  double cos() const {
    if (!cosTValid) {
      cosT = ::cos(this->_theta);
      cosTValid = true;
    }
    return cosT;
  }

  double sin() const {
    if (!sinTValid) {
      sinT = ::sin(this->_theta);
      sinTValid = true;
    }
    return sinT;
  }

};

#endif
