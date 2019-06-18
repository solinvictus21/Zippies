
#include "PathMove.h"

//the distince in mm within which is to be considered "at the target" for the purpose of terminating the current driving command
#define LINEAR_POSITION_EPSILON                           25.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
#define ANGULAR_POSITION_EPSILON                  0.05235987755983d

void PathMove::start(unsigned long startTime, ZippyController* zippy)
{
  end();
  currentMoveIndex = 0;
  currentMove = moves[0];
  currentMove->start(startTime, zippy);
}

unsigned long PathMove::loop(unsigned long currentTime, ZippyController* zippy)
{
  unsigned long remainingTime = currentMove->loop(currentTime, zippy);
  if (!remainingTime)
    return 0;

  //end the current move
  currentMove->end();

  //start the next move
  currentMoveIndex++;
  if (currentMoveIndex == moveCount)
    return remainingTime;

  currentMove = moves[currentMoveIndex];
  currentMove->start(currentTime - remainingTime, zippy);
  return 0;
}
