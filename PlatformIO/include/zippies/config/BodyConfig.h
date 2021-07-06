
#ifndef _BODYCONFIG_H_
#define _BODYCONFIG_H_

#define PLATFORM_TINYSCREEN                        1
// #define PLATFORM_TINYZERO                          1

//the diodes are ~39.7mm above the floor
//the height of the lighthouse above the plane of the diodes on the top of the Zippy
//from the top of the entertainment center
// #define LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z             908.0
//from the top of the TV
// #define LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z             1962.0
//on mic stand
#define LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z             1340.0

//the Y offset of the center of the Zippy body from the center of the sensors
//note that the sensors are assumed to be centered left/right, so there is no equivalent X offset
// #define BODY_CENTER_SENSOR_CENTER_OFFSET_Y                4.5

//the X/Y offset of the wheels
//note that code in the Zippy class assumes that the right wheel is the wheel that is offset forward
// #define WHEEL_CENTER_BODY_CENTER_OFFSET_X                17.3
// #define WHEEL_CENTER_BODY_CENTER_OFFSET_Y                 5.9

//new body redesign
#define BODY_CENTER_SENSOR_CENTER_OFFSET_Y                4.4
#define WHEEL_CENTER_BODY_CENTER_OFFSET_X                17.1
#define WHEEL_CENTER_BODY_CENTER_OFFSET_Y                 6.1

#endif
