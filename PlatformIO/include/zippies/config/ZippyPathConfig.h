
#ifndef _ZIPPYPATHCONFIG_H_
#define _ZIPPYPATHCONFIG_H_

#include "zippies/math/ZCubicHermiteSpline.h"

//follow the points and timings provided beyond this point
//parameters: the waypoint index of the anchor point for the path to follow
#define COMMAND_MOVE         0
//pin in place but look at points moving along another path
//parameters: the waypoint index of the anchor point for the path to watch
#define COMMAND_WATCH        1
//pin in place but mimic the orientations moving along another path
//parameters: the waypoint index of the anchor point for the path to mimic
#define COMMAND_MIMIC        2
//hold at the last target waypoint
//parameters: the amount of time in ms to pause
#define COMMAND_PAUSE        3
//follow points in forward direction
//parameters:
//  the timing start index
//  the number of timings to follow
#define COMMAND_FORWARD      4
//follow points in reverse
//parameters:
//  the timing start index
//  the number of timings to follow
#define COMMAND_REVERSE      5

typedef struct _ZippyWaypointTiming
{
    int index;
    unsigned long timing;
} ZippyWaypointTiming;

/*
void getZippyRoutine(
    ZMatrix2* startPosition,
    ZippyWaypoint** keyframes,
    int* keyframeCount);
*/

void getZippyRoutine(
    ZippyWaypoint** waypoints,
    ZippyWaypointTiming** timings,
    int** commands,
    int* commandCount);

#endif