
#ifndef _ZIPPYROUTINES_H_
#define _ZIPPYROUTINES_H_

#include <cstdint>

extern int KNOWN_ZIPPY_COUNT;

typedef struct _ZippyRoutinePoint
{
    int x, y, o;
} ZippyRoutinePoint;

typedef struct _ZippyRoutineData
{
    //starting point
    int startingPosition[3];
    //movements
    int* routineMovements;
    int routineMovementCount;
} ZippyRoutineData;

typedef struct _ZippyMotorConfig
{
    double motorStictionPower;
    double motorMinPower;
    double motorGain;
} ZippyMotorConfig;

typedef struct _ZippyConfig
{
    int zippyNumber;
    uint32_t mcuId[4];
    ZippyMotorConfig leftMotorConfig;
    ZippyMotorConfig rightMotorConfig;
    ZippyRoutineData* routine;
} ZippyConfig;

int getZippyNumber();
const ZippyConfig* getZippyConfig();
const ZippyConfig* getZippyConfig(int zippyNumber);
const ZippyConfig* getZippyTestConfig();
const ZippyConfig* getZippyTestConfig(int zippyNumber);

#endif
