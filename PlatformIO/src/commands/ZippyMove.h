
#ifndef _ZIPPYMOVE_H_
#define _ZIPPYMOVE_H_

#include "../Zippy.h"
#include "../lighthouse/KPosition.h"

class ZippyMove
{

public:
  ZippyMove()
  {}

  virtual unsigned long start(Zippy* zippy, const KPosition* sp) = 0;

  virtual void update(Zippy* zippy, double atNormalizedTime) const = 0;

  virtual void end() {
  }

  virtual ~ZippyMove()
  {}

};

#endif
