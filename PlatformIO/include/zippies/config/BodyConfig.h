
#ifndef _BODYCONFIG_H_
#define _BODYCONFIG_H_

#define PLATFORM_TINYSCREEN                        1
// #define PLATFORM_TINYZERO                          1

//the diodes are ~39.7mm above the floor
//the height of the lighthouse above the plane of the diodes on the top of the Zippy
//from the top of the entertainment center
// #define LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z             908.0d
//from the top of the TV
#define LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z             1962.0d
//the Y offset of the center of the Zippy body from the center of the sensors
//note that the sensors are assumed to be centered left/right, so there is no equivalent X offset
#define BODY_CENTER_SENSOR_CENTER_OFFSET_Y                4.2d
#define WHEEL_CENTER_BODY_CENTER_OFFSET_X                16.7d
#define WHEEL_CENTER_BODY_CENTER_OFFSET_Y                 5.9d
#define WHEEL_CENTER_BODY_CENTER_OFFSET                  17.711578134090706d
#define WHEEN_CENTER_BODY_CENTER_ANGLE                   19.457979011589846d

#endif
