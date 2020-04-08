
#ifndef _ZIPPYROUTINE_H_
#define _ZIPPYROUTINE_H_

#include "Path.h"
#include "zippies/ZippyMath.h"

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
} MovementState;

typedef struct _RoutineDefinition
{
  unsigned long timing;
  double easeInFactor;
  double easeOutFactor;
  int pathSegmentCount;
  PathDefinition* pathSegments;
  int pathRepeatCount;
} RoutineDefinition;

double getRoutineLength(const RoutineDefinition* routine, int routineSegmentCount);

class Routine
{

private:
  const RoutineDefinition* routineSegments;
  int routineSegmentCount = 0;
  int currentRoutineSegmentIndex = 0;
  MovementState currentMovementState = MovementState::Stopped;

  unsigned long currentRoutineSegmentStartTime = 0;
  int currentRoutineSegmentLoopCount = 0;

  void planCurrentRoutineSegment(unsigned long currentTime);
  unsigned long currentRoutineSegmentCompleted(unsigned long currentTime);
  Path path;

  KMatrix2 currentTargetPosition;

public:
  Routine() {}

  void setRoutineSegments(
      const KMatrix2* anchorPosition,
      RoutineDefinition* routineSegments,
      int routineSegmentCount);
  void reset();
  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  bool isRoutineCompleted() { return currentRoutineSegmentIndex >= routineSegmentCount; }

  const KMatrix2* getTargetPosition() const { return &currentTargetPosition; };
  MovementState getTargetMovementState() { return currentMovementState; }

};

void reverseRoutine(
  RoutineDefinition* routine,
  int routineCount);

#endif