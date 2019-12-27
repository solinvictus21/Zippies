
#ifndef _PATHINGCONFIG_H_
#define _PATHINGCONFIG_H_

#include "ZippyID.h"

//the X offset to path the Zippy left/right from the lighthouse center
//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
#if ZIPPY_ID == 0
#define PATHING_OFFSET_X                            60.0d
#elif ZIPPY_ID == 1
#define PATHING_OFFSET_X                             0.0d
#else
#define PATHING_OFFSET_X                           -60.0d
#endif

//the Y offset to path the Zippy forward/backward from the lighthouse center
// #define PATHING_OFFSET_Y                           940.0d
#define PATHING_OFFSET_Y                          1100.0d

#endif
