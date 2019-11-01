
#ifndef _ZROUTINECONTROLLER_H_
#define _ZROUTINECONTROLLER_H_

#include "ZController.h"
#include "lighthouse/SensorFusor.h"
#include "Zippy.h"
#include "paths/ZPath.h"
#include "lighthouse/KMatrix2.h"

class ZRoutineController : public ZController
{

private:
  Zippy* zippy;

  // int routineIndex = 0;
  ZippyRoutine routine;
  const Command* currentCommand = NULL;
  unsigned long currentCommandStartTime = 0;
  const ZPath* currentCommandPath = NULL;
  KMatrix2 currentTargetPosition;

  void loopRoutine(unsigned long currentTime);
  bool planNextCommand(unsigned long currentTime);
  unsigned long currentCommandCompleted(unsigned long currentTime);
  void pushCurrentPathPosition(double interpolatedTime);
  void loopZippy(unsigned long currentTime);

public:
  ZRoutineController(Zippy* z);

  void setStartPosition(const KMatrix2* startPosition) { currentTargetPosition.set(startPosition); }
  void start(unsigned long startTime);
  void setRoutine(Command* c, int cc, int lc) { routine.setCommands(c, cc, lc); }
  void loop(unsigned long currentTime);
  bool isRoutineCompleted() { return routine.isCompleted(); }
  void stop();

  ~ZRoutineController() {
    if (currentCommandPath)
      delete currentCommandPath;
  }

};

#endif
