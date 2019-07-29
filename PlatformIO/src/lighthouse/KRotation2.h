
#ifndef _KROTATION2_H_
#define _KROTATION2_H_

#include <Arduino.h>

class KRotation2
{

private:
  double _theta;
  mutable double cosT;
  mutable bool cosTValid = false;
  mutable double sinT;
  mutable bool sinTValid = false;

public:
  KRotation2(double t)
    : _theta(t)
  {}

  double theta() const {
    return this->_theta;
  }

  void setTheta(double t) {
    this->_theta = t;
    cosTValid = false;
    sinTValid = false;
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
