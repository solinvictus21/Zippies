
#ifndef _ROUTINECONTROLLER_H_
#define _ROUTINECONTROLLER_H_

#include "ZippyController.h"
#include "zippies/paths/Routine.h"
#include "PathFollowingController.h"
#include "zippies/config/PIDConfig.h"
#include "zippies/math/StatisticsAccumulator.h"

typedef enum class _RoutineExecutionState
{
  PreSyncingWithPreamble,
  MovingIntoPlace,
  Executing,
  PostSyncingWithPreamble,
} RoutineExecutionState;

class RoutineController : public ZippyController
{

private:
  SensorFusor* sensors;
  KMatrix2 defaultRoutinesStartPosition;
  RoutineDefinition* defaultRoutines;
  int defaultRoutinesCount;

  Routine routine;
  PathFollowingController pathFollowingController;

  RoutineExecutionState executionState = RoutineExecutionState::PreSyncingWithPreamble;

  void createRelativePathDefinition(const KVector2* relativeMovement, PathDefinition* pathDefinition);
  void planMoveIntoPlace(const KMatrix2* currentPosition, const KMatrix2* startPosition);

public:
  RoutineController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
