
#ifndef _ZIPPYROUTINE_H_
#define _ZIPPYROUTINE_H_

#include "ZippyConfig.h"

#define COMMAND_PAUSE                  0
#define COMMAND_MOVE_ARC              10
#define COMMAND_MOVE_BIARC            11
#define COMMAND_TURN_INTERPOLATED     20
#define COMMAND_TURN_IMMEDIATE        21

#define TIMING_BEATS_0_5             300
#define TIMING_BEATS_1               600
#define TIMING_BEATS_1_5             900
#define TIMING_BEATS_2              1200
#define TIMING_BEATS_2_5            1500
#define TIMING_BEATS_3              1800
#define TIMING_BEATS_4              2400
#define TIMING_BEATS_5              3000
#define TIMING_BEATS_6              3600
#define TIMING_BEATS_7              4200
#define TIMING_BEATS_8              4800

typedef struct _Command
{
  unsigned long timing;
  double x, y, o;
} Command;

#if ZIPPY_ID == 0

Command ROUTINE[] = {
  //intro
  { TIMING_BEATS_3,     -50.0d,   500.0d,    M_PI },
  { TIMING_BEATS_4,      50.0d,     0.0d,    M_PI },
  { TIMING_BEATS_3,      50.0d,     0.0d, -M_PI_2 },
  { TIMING_BEATS_5,      50.0d,     0.0d, -M_PI_2 },
  { TIMING_BEATS_1,      50.0d,     0.0d,    0.0d },

  //turn; dance forward to right
  { TIMING_BEATS_1,      50.0d,     0.0d,    0.0d },
  { TIMING_BEATS_1,     150.0d,    75.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,   150.0d,    0.0d },
  { TIMING_BEATS_1,     150.0d,   225.0d,    0.0d },

  //pause; dance backward to left
  { TIMING_BEATS_1,     150.0d,   225.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,   150.0d,    0.0d },
  { TIMING_BEATS_1,     150.0d,    75.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,     0.0d,    0.0d },

  //pause; dance forward to the left
  { TIMING_BEATS_1,      50.0d,     0.0d,    0.0d },
  { TIMING_BEATS_1,     -50.0d,    75.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,   150.0d,    0.0d },
  { TIMING_BEATS_1,     -50.0d,   225.0d,    0.0d },

  //pause; dance backward to right
  { TIMING_BEATS_1,     -50.0d,   225.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,   150.0d,    0.0d },
  { TIMING_BEATS_1,     -50.0d,    75.0d,    0.0d },
  { TIMING_BEATS_1,      50.0d,     0.0d,    0.0d },

  //clockwise circle in reverse
  { TIMING_BEATS_0_5,   -25.0d,   -75.0d,  M_PI_2 },
  { TIMING_BEATS_0_5,  -100.0d,     0.0d,    M_PI },
  { TIMING_BEATS_0_5,   -25.0d,    75.0d, -M_PI_2 },
  // { TIMING_BEATS_0_5,    50.0d,     0.0d,    0.0d },

  //rush off backwards to the right
  { TIMING_BEATS_2_5,   500.0d,    75.0d, -M_PI_2 },

  //currently offscreen; turn forward and then ease around to the stage right
  { TIMING_BEATS_2_5,     0.0d,   500.0d, -M_PI_2 },
  { TIMING_BEATS_2_5,  -500.0d,     0.0d,    M_PI },
  { TIMING_BEATS_1,    -500.0d,     0.0d,  M_PI_2 },

  //enter from stage right to center
  { TIMING_BEATS_1_5,     0.0d,     0.0d,  M_PI_2 },
  { TIMING_BEATS_0_5,     0.0d,     0.0d,    0.0d },
  { TIMING_BEATS_0_5,     0.0d,     0.0d, -M_PI_2 },
  { TIMING_BEATS_0_5,     0.0d,     0.0d,    M_PI },
  { TIMING_BEATS_0_5,     0.0d,     0.0d,  M_PI_2 },
  { TIMING_BEATS_0_5,     0.0d,     0.0d,    0.0d },

};

int ROUTINE_POSITION_COUNT = (int)(sizeof(ROUTINE) / sizeof(Command));
#else

uint8_t COMMANDS = {
  COMMAND_PAUSE,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  COMMAND_TURN_INTERPOLATED,
  COMMAND_PAUSE,
  //turn; dance forward to right
  COMMAND_TURN_IMMEDIATE,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  //pause; dance backward to left
  COMMAND_PAUSE,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  //pause; dance forward to the left
  COMMAND_PAUSE,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  //pause; dance backward to right
  COMMAND_PAUSE,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,
  COMMAND_MOVE_BIARC,

};

unsigned long TIMINGS = {
  //intro
  TIMING_BEATS_8,
  TIMING_BEATS_4,
  TIMING_BEATS_3,
  TIMING_BEATS_1,
  //turn; dance forward to right
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  //pause; dance backward to left
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  //pause; dance forward to left
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  //pause; dance backward to right
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,
  TIMING_BEATS_1,

};

double POSITIONS = {
    -50.0d,    0.0d,   M_PI,
};

#endif

#endif
