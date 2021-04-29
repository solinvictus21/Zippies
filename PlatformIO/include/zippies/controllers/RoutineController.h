
#ifndef _ROUTINECONTROLLER_H_
#define _ROUTINECONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/paths/Routine.h"
#include "zippies/pursuit/PursuitController.h"
#include "zippies/config/PIDConfig.h"
#include "zippies/math/StatisticsAccumulator.h"
#include "zippies/math/ZCubicHermiteSpline.h"
#include "zippies/config/ZippyPathConfig.h"

typedef enum class _RoutineExecutionState
{
  MovingIntoPlace,
  SyncingWithPreamble,
  Executing,
} RoutineExecutionState;

class RoutineController : public ZippyController
{

private:
  SensorFusor* sensors;
  ZMatrix2 defaultRoutinesStartPosition;
  ZippyWaypoint* defaultRoutines;
  int defaultRoutinesCount;

  ZVector2 relativeTargetPosition;
  ZVector2 relativeTargetVelocity;

  ZCubicHermiteSpline path;
  PursuitController* pursuitController;

  RoutineExecutionState executionState = RoutineExecutionState::MovingIntoPlace;

  void calculateRelativeTargets();

public:
  RoutineController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

  ~RoutineController() {
    delete pursuitController;
  }

};

#endif
