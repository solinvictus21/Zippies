
#ifndef _KROTATION_H_
#define _KROTATION_H_

#include <Arduino.h>

class KRotation
{

private:
  double t;
  mutable double cosT;
  mutable bool cosTValid = false;
  mutable double sinT;
  mutable bool sinTValid = false;

public:
  KRotation(double theta)
    : t(theta)
  {}

  double theta() const {
    return t;
  }

  void setTheta(double theta) {
    this->t = theta;
    cosTValid = false;
    sinTValid = false;
  }

  double cosTheta() const {
    if (!cosTValid) {
      cosT = cos(t);
      cosTValid = true;
    }
    return cosT;
  }

  double sinTheta() const {
    if (!sinTValid) {
      sinT = sin(t);
      sinTValid = true;
    }
    return sinT;
  }

};

#endif
