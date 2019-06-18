
#ifndef _ZIPPYMOVE_H_
#define _ZIPPYMOVE_H_

#include "../lighthouse/KPosition.h"
#include "../ZippyController.h"

class ZippyMove
{

protected:
  ZippyMove() {}

public:
  virtual void start(unsigned long st, ZippyController* zippy) = 0;
  virtual unsigned long loop(unsigned long currentTime, ZippyController* zippy) = 0;
  virtual void end() {}

  virtual ~ZippyMove() {}

};

#endif
