
#ifndef _EXECUTOR_H_
#define _EXECUTOR_H_

#include "ZippyMove.h"
#include "../lighthouse/Lighthouse.h"
#include "../Zippy.h"
#include "../lighthouse/KPosition.h"

enum ExecutorMode
{
  WaitForLighthouse, InitialPause, MoveIntoPlace, TurnIntoPlace, SyncWithPreamble, Executing
};

class Executor
{

private:
  double startingPositionX;
  double startingPositionY;
  double startingOrientation;
  ZippyMove** moves;
  int moveCount;

  Lighthouse lighthouse;
  Zippy zippy;

  unsigned long lastUpdateTime = 0;

  ExecutorMode currentMode = WaitForLighthouse;
  int currentMoveIndex = 0;
  ZippyMove* currentMove;
  KPosition currentMoveStartPosition;
  unsigned long currentMoveStartTime  = 0;
  unsigned long currentMoveDeltaTime  = 0;

  void startMove(unsigned long startTime, ZippyMove* move, const KPosition* startingPosition);
  void processCurrentMove(unsigned long currentTime);

public:
  Executor(double startingPositionX, double startingPositionY, double startingOrientation, ZippyMove** m, int mc);

  const Zippy* getZippy() { return &zippy; }
  const Lighthouse* getLighthouse() { return &lighthouse; }

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);

};

#endif
