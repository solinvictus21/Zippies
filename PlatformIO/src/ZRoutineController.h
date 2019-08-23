
#ifndef _ZROUTINECONTROLLER_H_
#define _ZROUTINECONTROLLER_H_

#include "ZController.h"
#include "lighthouse/Lighthouse.h"
#include "Zippy.h"
#include "paths/ZPath.h"
#include "lighthouse/KMatrix2.h"

typedef enum _MovementState
{
  MovementStopped,
  MovementTurning,
  MovementMoving
} MovementState;

class ZRoutineController : public ZController
{

private:
  Lighthouse* lighthouse;
  bool lighthouseReady = false;

  Zippy* zippy;
  MovementState currentMovementState = MovementStopped;

  // int routineIndex = 0;
  ZippyRoutine routine;
  Command* currentCommand = NULL;
  unsigned long currentCommandStartTime = 0;
  const ZPath* currentCommandPath = NULL;
  KMatrix2 currentTargetPosition;

  void startRoutine(unsigned long startTime, const KMatrix2* startPosition);
  void planCurrentCommand(unsigned long currentTime);
  void executeCurrentPath(unsigned long currentTime);
  unsigned long currentCommandCompleted(unsigned long currentTime);
  void stopRoutine();

public:
  ZRoutineController(Lighthouse* lighthouse, Zippy* z);

  void loop(unsigned long currentTime);

  ~ZRoutineController() {
    if (currentCommandPath)
      delete currentCommandPath;
  }

};

#endif
