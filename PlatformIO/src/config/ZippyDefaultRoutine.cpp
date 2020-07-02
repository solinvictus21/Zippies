
#include "zippies/config/ZippyDefaultRoutine.h"
#include "zippies/config/ZippyDefaultPaths.h"
#include "zippies/config/ZippyConfig.h"
#include "zippies/paths/Path.h"

// #define DEFAULT_PATH_TIMING                      42000
#define DEFAULT_PATH_TIMING                      60000

#define PATH_OFFSET_Y                             1050.0
#define PATH_SEPARATION_X                           60.0

extern PathDefinition START_FOLLOWING_PATHS[];
extern PathDefinition END_FOLLOWING_PATHS[];
extern RoutineDefinition DEFAULT_ROUTINES[];
extern const int DEFAULT_ROUTINES_COUNT;

void getZippyDefaultRoutines(
    ZMatrix2* startPosition,
    RoutineDefinition** routines,
    int* routineCount)
{
  int separationCount = getTotalZippyCount() - 1;
  double totalSeparationX = separationCount * PATH_SEPARATION_X;
  int zippyNumber = getCurrentZippyNumber();
  double currentZippySeparationX = ((double)(separationCount - zippyNumber)) * PATH_SEPARATION_X;
  startPosition->set(
    currentZippySeparationX - (totalSeparationX / 2.0),
    PATH_OFFSET_Y,
    0.0);
    // M_PI);

  START_FOLLOWING_PATHS[0].params.p1 = currentZippySeparationX + (3.0 * PATH_SEPARATION_X);
  // END_FOLLOWING_PATHS[0].params.p1 = (1.5d * PATH_SEPARATION_X) + (zippyNumber * PATH_SEPARATION_X);
  END_FOLLOWING_PATHS[0].params.p1 = (5.0 * FOLLOW_RADIUS_LARGE) - currentZippySeparationX - (3.0 * PATH_SEPARATION_X);

  // unsigned long pathFollowingDelay = 600 + (200 * (separationCount - zippyNumber));
  unsigned long pathFollowingDelay = (DEFAULT_PATH_TIMING/100.0) + (300.0 * (separationCount - zippyNumber));
  DEFAULT_ROUTINES[1].timing = pathFollowingDelay;
  // DEFAULT_ROUTINES[3].timing = 3000 - pathFollowingDelay;
  DEFAULT_ROUTINES[3].timing = (DEFAULT_PATH_TIMING/10) - pathFollowingDelay;
  *routines = &DEFAULT_ROUTINES[0];
  *routineCount = DEFAULT_ROUTINES_COUNT;
}

PathDefinition START_FOLLOWING_PATHS[] = {
  { PathDefinitionType::Move , PATH_SEPARATION_X },
};
const int START_FOLLOWING_PATHS_COUNT = sizeof(START_FOLLOWING_PATHS) / sizeof(PathDefinition);

PathDefinition END_FOLLOWING_PATHS[] = {
  { PathDefinitionType::Move ,   3.0 * PATH_SEPARATION_X   },
  // +200x, -200y, west
  { PathDefinitionType::Arc  ,       100.0,   M_PI_2       },
  // +100x, -100y, north
  { PathDefinitionType::Move ,       150.0                 },
  // +100x,    0y, north
};
const int END_FOLLOWING_PATHS_COUNT = sizeof(END_FOLLOWING_PATHS) / sizeof(PathDefinition);

RoutineDefinition DEFAULT_ROUTINES[] = {
  //first routine; demonstrate perfect synchronization along identical paths equally spaced apart
  { ((unsigned long)(DEFAULT_PATH_TIMING * 0.5)), 0.28, 0.33,      synchronizedPathLength,        synchronizedPath, 0 },

  //second routine; demonstrate ability to follow each other in extremely tight proximity without collisions
  {                            1000, 0.33, 0.33,  START_FOLLOWING_PATHS_COUNT,  START_FOLLOWING_PATHS, 0 },
  { ((unsigned long)(DEFAULT_PATH_TIMING * 0.5)), 0.33, 0.30,             followPathLength,             followPath, 0 },
  {                            3000, 0.33, 0.20,    END_FOLLOWING_PATHS_COUNT,    END_FOLLOWING_PATHS, 0 },

  //pause
  {   4000, 0.00, 0.00,                            0,                   NULL, 0 },
};
const int DEFAULT_ROUTINES_COUNT = sizeof(DEFAULT_ROUTINES) / sizeof(RoutineDefinition);
