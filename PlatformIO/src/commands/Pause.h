
#ifndef _PAUSE_H_
#define _PAUSE_H_

#include "ZippyCommand.h"
// #include "../Zippy.h"

class Pause : public ZippyCommand
{

public:
  Pause(unsigned long pt)
    : ZippyCommand(pt)
  {}

};

#endif
