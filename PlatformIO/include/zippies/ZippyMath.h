
#ifndef _ZIPPYMATH_H_
#define _ZIPPYMATH_H_

#include <Arduino.h>

#define M2_PI    6.283185307179586
#define M2_PI_34 4.712388980384690
const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

//2D math objects
#include "zippies/math/ZMatrix2.h"
#include "zippies/math/ZRotation2.h"
#include "zippies/math/ZVector2.h"

//3D math objects
#include "zippies/math/ZQuaternion3.h"
#include "zippies/math/ZVector3.h"

#endif
