
#ifndef _TURNINPLACE_H_
#define _TURNINPLACE_H_

#include "ZippyCommand.h"

class TurnInPlace : public ZippyCommand
{

private:
  double startingOrientation;
  double endingOrientation;

public:
  TurnInPlace(double so, double eo, unsigned long et)
    : ZippyCommand(et),
      startingOrientation(so),
      endingOrientation(eo)
  {}

  void setStartingOrientation(double orientation) {
    startingOrientation = orientation;
  }

  void setEndingOrientation(double orientation) {
    endingOrientation = orientation;
  }

  unsigned long getExecutionTime() {
    return executionTime;
  }

  void getOrientation(unsigned long atDeltaTime, double* targetOrientation) {
    double interpolation = ((double)atDeltaTime) / ((double)executionTime);
    *targetOrientation = startingOrientation + ((endingOrientation - startingOrientation) * interpolation);
  }

};

#endif
