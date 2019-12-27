
#ifndef _ZIPPYROUTINE_H_
#define _ZIPPYROUTINE_H_

#define TIMING_BEATS_0_25            600
#define TIMING_BEATS_0_5            1200
#define TIMING_BEATS_1_0            2400
#define TIMING_BEATS_1_5            3600
#define TIMING_BEATS_2_0            4800
#define TIMING_BEATS_2_5            6000
#define TIMING_BEATS_3_0            7200
#define TIMING_BEATS_4_0            9600
#define TIMING_BEATS_5_0           12000
#define TIMING_BEATS_6_0           14400
#define TIMING_BEATS_7_0           16800
#define TIMING_BEATS_8_0           19200


#define M_PI_14 0.785398163397448d
#define M_PI_34 2.356194490192345d

typedef enum class _MovementType
{
  //all commands below this point are relative to the current position and orientation
  Move,         //straight move forward or backward
  Turn,         //arc with no radius
  Arc,          //arc with radius and subtended angle
} MovementType;

typedef struct _Movement
{
  MovementType type;
  struct {
    double p1, p2;
  } params;
} Movement;

typedef struct _PathSegment
{
  unsigned long timing;
  double easeInFactor;
  double easeOutFactor;
  int movementCount;
  Movement* movements;
  int movementRepeatCount;
} PathSegment;

#endif
