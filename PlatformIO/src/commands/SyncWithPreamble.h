
#ifndef _SYNCWITHPREAMBLE_H_
#define _SYNCWITHPREAMBLE_H_

#include "ZippyCommand.h"
#include "Zippy.h"

class SyncWithPreamble : public ZippyCommand
{
private:
  Zippy* zippy;
  Lighthouse* lighthouse;

public:
  SyncWithPreamble(Zippy* zippy);
  void start(unsigned long currentTime);
  bool loop(unsigned long currentTime);

};

#endif
