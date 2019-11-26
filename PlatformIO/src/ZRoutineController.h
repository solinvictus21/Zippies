
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

  /*
  ZippyRoutine routine;
  const Command* currentCommand = NULL;
  unsigned long currentCommandStartTime = 0;
  const ZPath* currentCommandPath = NULL;
  */

  //first phase; bind the routine
  const Command2* commands;
  int commandCount = 0;
  int currentCommand = 0;

  //second phase; plan the current command
  double currentCommandLength = 0.0d;
  double currentCommandPosition = 0.0d;
  unsigned long currentCommandStartTime = 0;

  void planCurrentCommand(unsigned long currentTime);
  unsigned long currentCommandCompleted(unsigned long currentTime);
  double getMovementLength(const Movement* movement);

  //third phase; plot the current movement within the current command
  int currentMovementIndex = 0;
  int currentMovementLoopCount = 0;
  const ZPath* currentMovement = NULL;

  void plotCurrentMovement();

  //fourth phase; update the target position within the current movement
  KMatrix2 currentTargetPosition;

  void pushCurrentPathPosition(double interpolatedTime);

public:
  ZRoutineController(Zippy* z);

  void setAnchorPosition(const KMatrix2* startPosition) { currentTargetPosition.set(startPosition); }
  void setAnchorPosition(double x, double y, double o) { currentTargetPosition.set(x, y, o); }
  void start(unsigned long startTime);
  // void setRoutine(Command* c, int cc, int lc) { routine.setCommands(c, cc, lc); }
  void setRoutine(Command2* r, int rl);
  void loop(unsigned long currentTime);
  // bool isRoutineCompleted() { return routine.isCompleted(); }
  bool isRoutineCompleted() { return currentCommand >= commandCount; }
  void stop();

  ~ZRoutineController() {
    // if (currentCommandPath)
      // delete currentCommandPath;
    if (currentMovement)
      delete currentMovement;
  }

};

#endif
