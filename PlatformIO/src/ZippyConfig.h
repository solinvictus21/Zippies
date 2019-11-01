
#ifndef _ZIPPYCONFIG_H_
#define _ZIPPYCONFIG_H_

//0 - 8A95E7ED 31C99603 D0720874 3E6E2F70
//1 - EDE7958A 0396C931 740872D0 702F6E3E
#define ZIPPY_ID                                  0

#if ZIPPY_ID == 0
#define ZIPPY_OFFSET_X                          -30.0d
#else
#define ZIPPY_OFFSET_X                           30.0d
#endif

// #define ENABLE_BLUETOOTH                          1
// #define ENABLE_SDCARD_LOGGING                     1
// #define DEBUG_PID_ERROR                           1

#define PLATFORM_TINYSCREEN                       1
// #define PLATFORM_TINYZERO                         1

#define LIGHTHOUSE_CENTER_OFFSET_X                           0.0d
#define LIGHTHOUSE_CENTER_OFFSET_Y                         950.0d

#define WHEEL_OFFSET_X                           16.7d
#define WHEEL_OFFSET_Y                            5.9d

#define DOUBLE_EPSILON                            0.00001d

#endif
