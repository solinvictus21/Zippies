
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/config/ZippyConfig.h"

#define DEFAULT_PATH_SEPARATION_X                           80.0
#define DEFAULT_PATH_OFFSET_Y                              980.0
#define DEFAULT_TIME_FACTOR                                300

extern ZippyWaypoint zippyPath[];
extern const int zippyPathCount;

void getZippyRoutine(
    ZMatrix2* startPosition,
    ZippyWaypoint** keyframes,
    int* keyframeCount)
{
    int separationCount = getTotalZippyCount() - 1;
    double totalSeparationX = separationCount * DEFAULT_PATH_SEPARATION_X;
    int zippyNumber = getCurrentZippyNumber();
    double currentZippySeparationX = ((double)(separationCount - zippyNumber)) * DEFAULT_PATH_SEPARATION_X;
    double offsetX = currentZippySeparationX - (totalSeparationX / 2.0);
    for (int i = 0; i < zippyPathCount; i++)
        zippyPath[i].x += offsetX;

    startPosition->set(
        offsetX,
        DEFAULT_PATH_OFFSET_Y,
        0.0);
    *keyframes = &zippyPath[0];
    *keyframeCount = zippyPathCount;
}

ZippyWaypoint zippyPath[]
{
    {    0.0, DEFAULT_PATH_OFFSET_Y+   0.0,    0.0, DEFAULT_TIME_FACTOR* 4 },
    {  300.0, DEFAULT_PATH_OFFSET_Y+ 300.0,   90.0, DEFAULT_TIME_FACTOR* 4 },
    {  600.0, DEFAULT_PATH_OFFSET_Y+   0.0,  180.0, DEFAULT_TIME_FACTOR* 4 },
    {  300.0, DEFAULT_PATH_OFFSET_Y -300.0,  -90.0, DEFAULT_TIME_FACTOR* 8 },
    { -300.0, DEFAULT_PATH_OFFSET_Y+ 300.0,  -90.0, DEFAULT_TIME_FACTOR* 4 },
    { -600.0, DEFAULT_PATH_OFFSET_Y+   0.0,  180.0, DEFAULT_TIME_FACTOR* 4 },
    { -300.0, DEFAULT_PATH_OFFSET_Y -300.0,   90.0, DEFAULT_TIME_FACTOR* 8 },
    {  300.0, DEFAULT_PATH_OFFSET_Y+ 300.0,   90.0, DEFAULT_TIME_FACTOR* 8 },
    {  300.0, DEFAULT_PATH_OFFSET_Y -300.0,  -90.0, DEFAULT_TIME_FACTOR* 8 },
    { -300.0, DEFAULT_PATH_OFFSET_Y+ 300.0,  -90.0, DEFAULT_TIME_FACTOR* 8 },
    { -300.0, DEFAULT_PATH_OFFSET_Y -300.0,   90.0, DEFAULT_TIME_FACTOR* 4 },
    {    0.0, DEFAULT_PATH_OFFSET_Y+   0.0,    0.0 },
};

const int zippyPathCount = sizeof(zippyPath) / sizeof(ZippyWaypoint);

