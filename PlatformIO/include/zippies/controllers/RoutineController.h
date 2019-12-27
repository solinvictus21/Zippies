
#ifndef _ROUTINECONTROLLER_H_
#define _ROUTINECONTROLLER_H_

#include "ZippyController.h"
#include "Routine.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/Zippy.h"
#include "paths/ZPath.h"
#include "zippies/ZippyMath.h"

class RoutineController : public ZippyController
{

private:
  Zippy* zippy;

  //first phase; bind the routine
  const PathSegment* commands;
  int commandCount = 0;
  int currentCommandIndex = 0;

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
  double currentMovementLength = 0.0d;

  void plotCurrentMovement();

  //fourth phase; update the target position within the current movement
  KMatrix2 currentTargetPosition;

  void pushCurrentPathPosition(double interpolatedTime);

public:
  RoutineController(Zippy* z);

  void setAnchorPosition(const KMatrix2* startPosition) { currentTargetPosition.set(startPosition); }
  void setAnchorPosition(double x, double y, double o) { currentTargetPosition.set(x, y, o); }
  void start(unsigned long startTime);
  void setRoutine(PathSegment* r, int rl);
  void loop(unsigned long currentTime);
  bool isRoutineCompleted() { return currentCommandIndex >= commandCount; }
  void stop();

  ~RoutineController() {
    if (currentMovement)
      delete currentMovement;
  }

};

#endif
