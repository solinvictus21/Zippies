
#include "Executor.h"

//the number of milliseconds between each time we evaluate the current position of the Zippy and adjust its motors to stay on the path
#define LOOP_INTERVAL_MS                         40

//the time to pause between the moment the lighthouse signal is detected after it is lost and the moment we start moving again
//adding an initial pause before moving allows time to completely set the Zippy down and the position detection to stabilize
#define INITIAL_PAUSE_TIME                     2000

Executor::Executor(double spx, double spy, double so, ZippyMove** m, int mc)
  : startingPositionX(spx),
    startingPositionY(spy),
    startingOrientation(so),
    moves(m),
    moveCount(mc),
    zippy(LOOP_INTERVAL_MS)
{
}

void Executor::start(unsigned long currentTime)
{
  //start the lighthouse
  lighthouse.start();
  lastUpdateTime = currentTime;
}

void Executor::loop(unsigned long currentTime)
{
  //always process the Lighthouse diode hits
  lighthouse.loop(currentTime);

  if (currentMoveIndex >= moveCount || currentTime - lastUpdateTime < LOOP_INTERVAL_MS)
    return;
  lastUpdateTime += LOOP_INTERVAL_MS;

  //for debugging, to provide a clear indication when our processing is falling behind
  if (lastUpdateTime != currentTime)
    zippy.getFace()->clearScreen();

  if (!lighthouse.recalculate(currentTime)) {
    //we are no longer able to determine our current position; stop moving
    if (currentMode > WaitForLighthouse) {
      zippy.stop();
      currentMode = WaitForLighthouse;
    }
    return;
  }

  const KPosition* currentPosition = lighthouse.getPosition();
  const KPosition* currentPositionDelta = lighthouse.getPositionDelta();
  switch (currentMode) {
    case WaitForLighthouse:
      currentMoveStartTime = currentTime;
      currentMode = InitialPause;
    case InitialPause:
      if (currentTime - currentMoveStartTime >= INITIAL_PAUSE_TIME) {
        //move to the starting position
        zippy.start();
        zippy.setReverse(false);
        zippy.move(startingPositionX, startingPositionY,
            atan2(startingPositionX - currentPosition->vector.getX(), startingPositionY - currentPosition->vector.getY()));
        currentMode = MoveIntoPlace;
        /*
        //determine the 16-bit ID of this Zippy
        SerialUSB.print("Zippy ID: ");
        // uint32_t* zippyID = ((uint32_t*)0x0080A00C);
        // for (int i = 0; i < 4; i++)
          // SerialUSB.print(zippyID[i], HEX);
        uint8_t* zippyID = ((uint8_t*)0x0080A00C);
        for (int i = 0; i < 16; i++) {
          if (zippyID[i] < 0x10)
            SerialUSB.print("0");
          SerialUSB.print(zippyID[i], HEX);
        }
        SerialUSB.println();
        */
      }
      break;
    case MoveIntoPlace:
      if (zippy.loop(currentPosition, currentPositionDelta)) {
        //turn toward our starting orientation
        zippy.turn(startingOrientation);
        currentMode = TurnIntoPlace;
      }
      break;
    case TurnIntoPlace:
      if (zippy.loop(currentPosition, currentPositionDelta)) {
        //stop moving and sync with the Lighthouse preamble
        zippy.stop();
        lighthouse.clearPreambleFlag();
        currentMode = SyncWithPreamble;
      }
      break;
    case SyncWithPreamble:
      zippy.loop(currentPosition, currentPositionDelta);
      if (lighthouse.foundPreamble()) {
        //start executing moves
        currentMoveIndex = 0;
        startMove(currentTime, moves[currentMoveIndex], currentPosition);
        currentMode = Executing;
      }
      break;
    case Executing:
      processCurrentMove(currentTime);
      zippy.loop(currentPosition, currentPositionDelta);
      break;
  }
}

void Executor::startMove(unsigned long startTime, ZippyMove* move, const KPosition* startingPosition)
{
  currentMove = move;
  currentMoveStartTime = startTime;
  currentMoveStartPosition.vector.set(&startingPosition->vector);
  currentMoveStartPosition.orientation = startingPosition->orientation;
  currentMoveDeltaTime = currentMove->start(&zippy, &currentMoveStartPosition);
}

void Executor::processCurrentMove(unsigned long currentTime)
{
  unsigned long deltaTime = currentTime - currentMoveStartTime;
  if (deltaTime >= currentMoveDeltaTime) {
    //end the current move
    deltaTime -= currentMoveDeltaTime;
    currentMove->update(&zippy, 1.0d);
    currentMove->end();

    //start the next move
    // currentMoveIndex = (currentMoveIndex + 1) % moveCount;
    currentMoveIndex++;
    if (currentMoveIndex == moveCount) {
      currentMoveIndex = 0;
      // currentMode = WaitForLighthouse;
      // return;
    }
    startMove(currentTime - deltaTime, moves[currentMoveIndex], zippy.getTargetPosition());
  }

  //determine the current target position and orientation
  double interpolation = ((double)deltaTime) / ((double)currentMoveDeltaTime);
  currentMove->update(&zippy, interpolation);
}
