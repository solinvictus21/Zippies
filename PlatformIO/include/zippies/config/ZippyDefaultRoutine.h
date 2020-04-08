
#ifndef _ZIPPYDEFAULTROUTINE_H_
#define _ZIPPYDEFAULTROUTINE_H_

#include "zippies/math/KMatrix2.h"
#include "zippies/paths/Routine.h"

void getZippyDefaultRoutines(
    KMatrix2* startPosition,
    RoutineDefinition** routines,
    int* routineCount);

#endif
