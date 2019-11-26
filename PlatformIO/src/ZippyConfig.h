
#ifndef _ZIPPYCONFIG_H_
#define _ZIPPYCONFIG_H_

#define ZIPPY_ID                                   2

#if ZIPPY_ID == 0

//0 - red
#define ZIPPY_OFFSET_X                            80.0d
#define DEFAULT_KP                               270.00d
#define DEFAULT_KI                                 0.00d
#define DEFAULT_KD                                23.00d

#elif ZIPPY_ID == 1

//1 - green
#define ZIPPY_OFFSET_X                             0.0d
#define DEFAULT_KP                               270.00d
#define DEFAULT_KI                                 0.00d
#define DEFAULT_KD                                23.00d

#else

//2 - blue
#define ZIPPY_OFFSET_X                           -80.0d
#define DEFAULT_KP                               270.00d
#define DEFAULT_KI                                 0.00d
#define DEFAULT_KD                                23.00d

#endif

// #define ENABLE_BLUETOOTH                          1
// #define ENABLE_SDCARD_LOGGING                     1

#define PLATFORM_TINYSCREEN                        1
// #define PLATFORM_TINYZERO                          1

#define LIGHTHOUSE_CENTER_OFFSET_X                 0.0d
#define LIGHTHOUSE_CENTER_OFFSET_Y               950.0d

#define WHEEL_OFFSET_X                            16.7d
#define WHEEL_OFFSET_Y                             5.9d

#define DOUBLE_EPSILON                             0.00001d

#endif
