
#ifndef _PIDCONFIG_H_
#define _PIDCONFIG_H_

#define PID_OUTPUT_LIMIT                   60000.0

// /*
//PID CONFIGURATION - no setpoint changes
#define PID_KP                               300.0
#define PID_KI                                 0.0
#define PID_KD                                36.0
// */

/*
//PID CONFIGURATION - with setpoint changes
#define PID_KP                               180.0
#define PID_KI                                 0.0
#define PID_KD                                30.0
// */

/*
//P on M
#define PID_KP                               200.0
#define PID_KI                                20.0
#define PID_KD                                10.0
*/

#endif
