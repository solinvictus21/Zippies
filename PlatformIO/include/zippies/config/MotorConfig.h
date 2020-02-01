
#ifndef _MOTORCONFIG_H_
#define _MOTORCONFIG_H_

#include "ZippyID.h"

//the minimum PCM value below which the motors do not turn at all; i.e. the "dead zone"
//the range of outputs from the PID over which to ramp through the motor dead zone
#if ZIPPY_ID == 0
// #define MOTOR_DEAD_ZONE_ABS                     3270.0d
#define MOTOR_DEAD_ZONE_ABS                     3900.0d
#elif ZIPPY_ID == 1
// #define MOTOR_DEAD_ZONE_ABS                     3090.0d
#define MOTOR_DEAD_ZONE_ABS                     3500.0d
#else
// #define MOTOR_DEAD_ZONE_ABS                     2890.0d
#define MOTOR_DEAD_ZONE_ABS                     3400.0d
#endif

#endif
