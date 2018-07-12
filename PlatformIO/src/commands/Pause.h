
#ifndef _PAUSE_H_
#define _PAUSE_H_

#include "ZippyCommand.h"
#include "../Zippy.h"

class Pause : public ZippyCommand
{

private:
  Zippy* zippy;
  unsigned long deltaTimeMS;
  unsigned long startTimeMS;

public:
  Pause(Zippy* zippy, double deltaTimeSeconds);
  void start(unsigned long currentTime);
  bool loop(unsigned long currentTime);

};

#endif
