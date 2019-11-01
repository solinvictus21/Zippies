
#ifndef _ZIPPYROUTINE_H_
#define _ZIPPYROUTINE_H_

#include "ZippyConfig.h"

/*
#define TIMING_BEATS_0_25            150
#define TIMING_BEATS_0_5             300
#define TIMING_BEATS_1_0             600
#define TIMING_BEATS_1_5             900
#define TIMING_BEATS_2_0            1200
#define TIMING_BEATS_2_5            1500
#define TIMING_BEATS_3_0            1800
#define TIMING_BEATS_4_0            2400
#define TIMING_BEATS_5_0            3000
#define TIMING_BEATS_6_0            3600
#define TIMING_BEATS_7_0            4200
#define TIMING_BEATS_8_0            4800
*/
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

typedef enum _CommandType
{
  CommandPause,        //do nothing
  CommandMoveTo,       //absolute move to a specific point and orientation
  CommandTurnTo,       //absolute turn to a specific orientation
  //all commands below this point are relative to the current position and orientation
  CommandMove,         //straight move forward or backward
  CommandArc,          //arc with radius and subtended angle
  CommandTurn,         //arc with no radius
  //all commands below this point are control structures
  CommandSubRoutine
} CommandType;

typedef enum _EasingType
{
  EasingNone,
  EasingIn,
  EasingOut,
  EasingInOut
} EasingType;

typedef struct _Command
{
  unsigned long timing;
  CommandType type;
  union {
    struct {
      double p1, p2, p3;
    } params;
    struct {
      _Command* subcommands;
      int subcommandsCount;
      int loopCount;
    } subroutine;
  };
  EasingType easing;
} Command;

typedef enum class _MovementType
{
  MoveTo,       //move to a specific position and orientation
  Move,         //straight move forward or backward
  Arc,          //arc with radius and subtended angle
  Turn,         //arc with no radius
} MovementType;

typedef struct _Movement
{
  MovementType movementType;
} Movement;

typedef struct _Routine
{
  double anchorX, anchorY, anchorZ;
  unsigned long timing;
  EasingType easing;
  int subroutineCount;
  Movement* subroutines;
} Routine;

class ZippyRoutine
{

private:
  Command* commands;
  int commandCount;
  int loopCount;

  int currentCommandIndex = -1;
  ZippyRoutine* currentSubroutine = NULL;
  int currentLoopCount = 0;

public:
  ZippyRoutine(Command* c, int cc)
    : commands(c),
      commandCount(cc),
      loopCount(0)
  {}

  ZippyRoutine(Command* c, int cc, int lc)
    : commands(c),
      commandCount(cc),
      loopCount(lc)
  {}

  void setCommands(Command* c, int cc, int lc)
  {
    this->commands = c;
    this->commandCount = cc;
    this->loopCount = lc;
    reset();
  }

  const Command* getNextCommand()
  {
    if (loopCount && currentLoopCount >= loopCount)
      return NULL;

    if (currentSubroutine) {
      const Command* nextCommand = currentSubroutine->getNextCommand();
      if (nextCommand)
        return nextCommand;

      //subroutine is complete
      delete currentSubroutine;
      currentSubroutine = NULL;
    }

    currentCommandIndex++;
    if (currentCommandIndex >= commandCount) {
      currentLoopCount++;
      if (currentLoopCount >= loopCount) {
        if (loopCount)
          return NULL;

        currentLoopCount = 0;
      }
      currentCommandIndex = 0;
    }

    // SerialUSB.print("Preparing command index: ");
    // SerialUSB.println(currentCommandIndex);
    Command* nextCommand = &commands[currentCommandIndex];
    if (nextCommand->type == CommandSubRoutine) {
      currentSubroutine = new ZippyRoutine(
        nextCommand->subroutine.subcommands,
        nextCommand->subroutine.subcommandsCount,
        nextCommand->subroutine.loopCount);

      return currentSubroutine->getNextCommand();
    }

    return nextCommand;
  }

  void reset() {
    if (currentSubroutine) {
      delete currentSubroutine;
      currentSubroutine = NULL;
    }
    currentCommandIndex = -1;
    currentLoopCount = 0;
  }

  bool isCompleted() { return loopCount && currentLoopCount >= loopCount; }

  ~ZippyRoutine() {
    if (currentSubroutine)
      delete currentSubroutine;
  }

};

extern Command ROUTINE[];
extern int ROUTINE_POSITION_COUNT;

#endif
