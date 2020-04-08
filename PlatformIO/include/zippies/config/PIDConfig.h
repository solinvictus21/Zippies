
#ifndef _PIDCONFIG_H_
#define _PIDCONFIG_H_

//forcing the PID to update less often can help with PID tuning because it will exaggerate issues with over-shoot
//due to too high of a proportional or over-correction due to too high of a derivative
// #define PID_UPDATE_INTERVAL                  100
// #define PID_UPDATE_INTERVAL                   33
// #define PID_UPDATE_INTERVAL                   17

//PID CONFIGURATION
#define PID_KP                               160.0d
#define PID_KI                                 0.0d
// #define PID_KD                                20.0d
#define PID_KD                                20.0d
#define PID_OUTPUT_LIMIT                   60000.0d

#endif
