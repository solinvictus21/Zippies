
#ifndef _ZIPPYCONFIG_H_
#define _ZIPPYCONFIG_H_

//0 - 8A95E7ED 31C99603 D0720874 3E6E2F70
//0 - EDE7958A 0396C931 740872D0 702F6E3E
#define ZIPPY_ID                                  0

#if ZIPPY_ID == 0
#define ZIPPY_OFFSET_X                          -50.0d
#else
#define ZIPPY_OFFSET_X                           50.0d
#endif

//the number of milliseconds between each time we evaluate the current position of the Zippy and adjust its motors
// #define LOOP_INTERVAL_MS                         25

// #define ENABLE_BLUETOOTH                          1
#define ENABLE_SDCARD_LOGGING                     1

#define PLATFORM_TINYSCREEN                       1
// #define PLATFORM_TINYZERO                         1
// #define PLATFORM_EXEN_MINI                        1

#define LIGHTHOUSE_CENTER_OFFSET_X                           0.0d
// #define LIGHTHOUSE_CENTER_OFFSET_Y                        1200.0d
#define LIGHTHOUSE_CENTER_OFFSET_Y                         950.0d

#define WHEEL_OFFSET_X                           16.7d
#define WHEEL_OFFSET_Y                            5.9d

#define DOUBLE_EPSILON                            0.000001d

#endif
