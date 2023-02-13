
#ifndef _ZIPPYROUTINES_H_
#define _ZIPPYROUTINES_H_

#include <Arduino.h>

#define ROUTINE_FLAG_WEIGH_ANCHOR        0b1000
#define ROUTINE_FLAG_DROP_ANCHOR         0b0100
#define ROUTINE_FLAG_INVERT_X            0b0010
#define ROUTINE_FLAG_INVERT_Y            0b0001
#define ROUTINE_FLAG_PAUSE                   -1

typedef struct _ZippyRoutinePoint
{
    int x, y, o;
} ZippyRoutinePoint;

typedef struct _ZippyRoutineData
{
    //zippy UUID
    uint32_t uuid[4];

    //starting point
    int startingPosition[3];

    //movements
    int* routineMovements;
    int routineMovementCount;
} ZippyRoutineData;

const ZippyRoutineData* getZippyRoutineData();

#endif