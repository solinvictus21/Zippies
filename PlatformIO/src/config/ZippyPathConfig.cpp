
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/config/ZippyConfig.h"

#define DEFAULT_PATH_SEPARATION_X                           80.0
#define DEFAULT_PATH_OFFSET_Y                              980.0
// #define DEFAULT_TIME_FACTOR                                300
#define DEFAULT_TIME_FACTOR                                300

#define ZIPPY_RED_OFFSET_X        160.0
#define ZIPPY_ORANGE_OFFSET_X      80.0
#define ZIPPY_GREEN_OFFSET_X        0.0
#define ZIPPY_BLUE_OFFSET_X       -80.0
#define ZIPPY_PURPLE_OFFSET_X    -160.0

extern ZippyWaypoint zippyPath[];
extern const int zippyPathCount;

extern ZippyWaypoint zippyWaypoints[];
extern ZippyWaypointTiming waypointTimings[];

typedef struct _ZippyRoutineConfig
{
    int* routine;
    int routineCount;
} ZippyRoutineConfig;
extern ZippyRoutineConfig zippyRoutines[];

void getZippyRoutine(
    ZippyWaypoint** waypoints,
    ZippyWaypointTiming** timings,
    int** commands,
    int* commandCount)
{
    *waypoints = &zippyWaypoints[0];
    *timings = &waypointTimings[0];

    int zippyNumber = getCurrentZippyNumber();
    *commands = zippyRoutines[zippyNumber].routine;
    *commandCount = zippyRoutines[zippyNumber].routineCount;
}

ZippyWaypoint zippyWaypoints[]
{
    { ZIPPY_RED_OFFSET_X           , DEFAULT_PATH_OFFSET_Y,           0.0 },
    { ZIPPY_ORANGE_OFFSET_X        , DEFAULT_PATH_OFFSET_Y,           0.0 },
    { ZIPPY_GREEN_OFFSET_X         , DEFAULT_PATH_OFFSET_Y,           0.0 },
    { ZIPPY_BLUE_OFFSET_X          , DEFAULT_PATH_OFFSET_Y,           0.0 },
    { ZIPPY_PURPLE_OFFSET_X        , DEFAULT_PATH_OFFSET_Y,           0.0 },
    {     0.0,     0.0,    0.0 }, //5
    {   300.0,   300.0,   90.0 },
    {   600.0,     0.0,  180.0 },
    {   300.0,  -300.0,  -90.0 },
    {  -300.0,   300.0,  -90.0 },
    {  -600.0,     0.0,  180.0 }, //10
    {  -300.0,  -300.0,   90.0 },
    {   150.0,   150.0,   90.0 },
    {   300.0,     0.0,  180.0 },
    {   150.0,  -150.0,  -90.0 },
    {  -150.0,   150.0,  -90.0 }, //15
    {  -300.0,     0.0,  180.0 },
    {  -150.0,  -150.0,   90.0 },
    {    80.0,    80.0,   90.0 },
    {   160.0,     0.0,  180.0 },
    {    80.0,   -80.0,  -90.0 }, //20
    {   -80.0,    80.0,  -90.0 },
    {  -160.0,     0.0,  180.0 },
    {   -80.0,   -80.0,   90.0 },

    //TBD - test moves
    {    50.0,   100.0,   53.0 }, //24
    {   -50.0,   100.0,  -53.0 },
};

ZippyWaypointTiming waypointTimings[]
{
    //double figure eight
    { 6, DEFAULT_TIME_FACTOR* 4 }, //0
    { 7, DEFAULT_TIME_FACTOR* 4 },
    { 8, DEFAULT_TIME_FACTOR* 4 },
    { 9, DEFAULT_TIME_FACTOR* 8 },
    { 10, DEFAULT_TIME_FACTOR* 4 },
    { 11, DEFAULT_TIME_FACTOR* 4 },
    { 6, DEFAULT_TIME_FACTOR* 8 },
    { 8, DEFAULT_TIME_FACTOR* 8 },
    { 9, DEFAULT_TIME_FACTOR* 8 },
    { 11, DEFAULT_TIME_FACTOR* 8 },
    { 5, DEFAULT_TIME_FACTOR* 4 },

    //single figure eight, reversed
    { 11, DEFAULT_TIME_FACTOR* 4 }, //11
    { 10, DEFAULT_TIME_FACTOR* 4 },
    { 9, DEFAULT_TIME_FACTOR* 4 },
    { 8, DEFAULT_TIME_FACTOR* 8 },
    { 7, DEFAULT_TIME_FACTOR* 4 },
    { 6, DEFAULT_TIME_FACTOR* 4 },
    { 5, DEFAULT_TIME_FACTOR* 4 },

    //circle left
    { 9, DEFAULT_TIME_FACTOR* 4 }, //18
    { 10 , DEFAULT_TIME_FACTOR* 4 },
    { 11, DEFAULT_TIME_FACTOR* 4 },
    { 5, DEFAULT_TIME_FACTOR* 4 },

    //circle right
    { 6, DEFAULT_TIME_FACTOR* 4 }, //22
    { 7, DEFAULT_TIME_FACTOR* 4 },
    { 8, DEFAULT_TIME_FACTOR* 4 },
    { 5, DEFAULT_TIME_FACTOR* 4 },

    //small circle left
    { 15, DEFAULT_TIME_FACTOR* 3 }, //26
    { 16 , DEFAULT_TIME_FACTOR* 3 },
    { 17, DEFAULT_TIME_FACTOR* 3 },
    { 5, DEFAULT_TIME_FACTOR* 3 },

    //small circle right
    { 12, DEFAULT_TIME_FACTOR* 3 }, //30
    { 13, DEFAULT_TIME_FACTOR* 3 },
    { 14, DEFAULT_TIME_FACTOR* 3 },
    { 5, DEFAULT_TIME_FACTOR* 3 },

    //smaller circle left
    { 21, DEFAULT_TIME_FACTOR* 2 }, //34
    { 22 , DEFAULT_TIME_FACTOR* 2 },
    { 23, DEFAULT_TIME_FACTOR* 2 },
    { 5, DEFAULT_TIME_FACTOR* 2 },

    //smaller circle right
    { 18, DEFAULT_TIME_FACTOR* 2 }, //38
    { 19, DEFAULT_TIME_FACTOR* 2 },
    { 20, DEFAULT_TIME_FACTOR* 2 },
    { 5, DEFAULT_TIME_FACTOR* 2 },

    //TBD - test moves
    { 5, DEFAULT_TIME_FACTOR* 2 }, //42
    { 24, DEFAULT_TIME_FACTOR* 2 }, 
    { 25, DEFAULT_TIME_FACTOR* 2 },
};

int redRoutine[]
{
    COMMAND_MOVE,      0,
    COMMAND_FORWARD,   0,   11,
    COMMAND_PAUSE, DEFAULT_TIME_FACTOR* 4,
    COMMAND_REVERSE,  11,    7,
    COMMAND_PAUSE, DEFAULT_TIME_FACTOR* 4,

    COMMAND_WATCH,     2,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      0,
    COMMAND_FORWARD,  18,    4,
    COMMAND_FORWARD,  18,    4,

    COMMAND_WATCH,     4,
    COMMAND_FORWARD,  22,    4,
    COMMAND_FORWARD,  22,    4,

    COMMAND_MOVE,      0,
    COMMAND_FORWARD,  38,   4,
};
const int redCommandCount = sizeof(redRoutine) / sizeof(int);

int orangeRoutine[]
{
    COMMAND_MOVE,      1,
    COMMAND_FORWARD,   0,   11,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,
    COMMAND_REVERSE,  11,    7,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,

    COMMAND_WATCH,     2,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  26,    4,

    COMMAND_WATCH,     3,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_WATCH,     4,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      1,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
};
const int orangeCommandCount = sizeof(orangeRoutine) / sizeof(int);

int greenRoutine[]
{
    /*
    COMMAND_MOVE,      2,
    COMMAND_FORWARD,  43,    1,
    COMMAND_PAUSE,   100,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,   100,
    COMMAND_FORWARD,  44,    1,
    COMMAND_PAUSE,   100,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,  1000,
    COMMAND_FORWARD,  43,    1,
    COMMAND_PAUSE,   150,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,   150,
    COMMAND_FORWARD,  44,    1,
    COMMAND_PAUSE,   150,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,  1000,
    COMMAND_FORWARD,  43,    1,
    COMMAND_PAUSE,   200,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,   200,
    COMMAND_FORWARD,  44,    1,
    COMMAND_PAUSE,   200,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,  1000,
    COMMAND_FORWARD,  43,    1,
    COMMAND_PAUSE,   300,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,   300,
    COMMAND_FORWARD,  44,    1,
    COMMAND_PAUSE,   300,
    COMMAND_REVERSE,  42,    1,
    COMMAND_PAUSE,   300,
    */

    // /*
    COMMAND_MOVE,      2,
    COMMAND_FORWARD,   0,   11,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,
    COMMAND_REVERSE,  11,    7,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,

    COMMAND_MOVE,      2,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  26,    4,

    COMMAND_WATCH,     3,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_WATCH,     4,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      2,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
    // */
};
const int greenCommandCount = sizeof(greenRoutine) / sizeof(int);

int blueRoutine[]
{
    COMMAND_MOVE,      3,
    COMMAND_FORWARD,   0,   11,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,
    COMMAND_REVERSE,  11,    7,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,

    COMMAND_WATCH,     2,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  26,    4,

    COMMAND_MOVE,      3,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_WATCH,     4,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      3,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
};
const int blueCommandCount = sizeof(blueRoutine) / sizeof(int);

int purpleRoutine[]
{
    COMMAND_MOVE,      4,
    COMMAND_FORWARD,   0,   11,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,
    COMMAND_REVERSE,  11,    7,
    COMMAND_PAUSE,     DEFAULT_TIME_FACTOR* 4,

    COMMAND_WATCH,     2,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  26,    4,

    COMMAND_WATCH,     3,
    COMMAND_FORWARD,  26,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      4,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  30,    4,

    COMMAND_MOVE,      4,
    COMMAND_FORWARD,  30,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
    COMMAND_FORWARD,  38,    4,
    COMMAND_FORWARD,  34,    4,
};
const int purpleCommandCount = sizeof(purpleRoutine) / sizeof(int);

ZippyRoutineConfig zippyRoutines[]
{
    { &redRoutine[0], redCommandCount },
    { &orangeRoutine[0], orangeCommandCount },
    { &greenRoutine[0], greenCommandCount },
    { &blueRoutine[0], blueCommandCount },
    { &purpleRoutine[0], purpleCommandCount },
};
