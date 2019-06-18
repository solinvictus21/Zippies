
#ifndef _PATHMOVE_H_
#define _PATHMOVE_H_

#include "ZippyMove.h"
#include "../lighthouse/KPosition.h"

class PathMove : public ZippyMove
{

private:
  ZippyMove** moves;
  int moveCount;

  int currentMoveIndex = 0;
  ZippyMove* currentMove = NULL;

public:
  PathMove(ZippyMove** m, int mc)
    : moves(m),
      moveCount(mc)
  {}

  void start(unsigned long st, ZippyController* zippy);
  unsigned long loop(unsigned long currentTime, ZippyController* zippy);

  void end() {
    if (currentMove != NULL) {
      currentMove->end();
      currentMove = NULL;
    }
  }

  ~PathMove()
  {
    for (int i = 0; i < moveCount; i++)
      delete moves[0];
    delete[] moves;
  }

};

#endif
