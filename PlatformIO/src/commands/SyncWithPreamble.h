
#ifndef _SYNCWITHPREAMBLE_H_
#define _SYNCWITHPREAMBLE_H_

#include "ZippyCommand.h"
#include "../lighthouse/Lighthouse.h"

class SyncWithPreamble : public ZippyCommand
{
private:
  Lighthouse* lighthouse;

public:
  SyncWithPreamble(Lighthouse* l)
    : lighthouse(l)
  {}

};

#endif
