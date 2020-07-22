
#ifndef _ROUTINECONTROLLER_H_
#define _ROUTINECONTROLLER_H_

#include "ZippyController.h"
#include "zippies/paths/Routine.h"
#include "PathFollowingController.h"
#include "zippies/config/PIDConfig.h"
#include "zippies/math/StatisticsAccumulator.h"
#include "zippies/math/ZCubicHermiteSpline.h"
#include "zippies/config/ZippyPathConfig.h"

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
  ZMatrix2 defaultRoutinesStartPosition;
  ZippyWaypoint* defaultRoutines;
  int defaultRoutinesCount;

  ZCubicHermiteSpline path;
  PathFollowingController pathFollowingController;

  RoutineExecutionState executionState = RoutineExecutionState::PreSyncingWithPreamble;

public:
  RoutineController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
