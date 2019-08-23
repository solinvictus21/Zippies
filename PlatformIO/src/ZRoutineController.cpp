
#include "ZippyConfig.h"
#include "ZRoutineController.h"
#include "paths/Turn.h"
#include "paths/Move.h"
#include "paths/Arc.h"
#include "paths/CompositePath.h"
#include "paths/ZPathPlanner.h"

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>

#define CHIP_SELECT_PIN        10
#define LOG_FILENAME           "log"
#define LOG_FILENAME_EXTENSION ".csv"
#define LOG_MAX_FILES          10
bool sdCardInserted = false;
File logFile;
int logNumber = 0;
#endif

//the distince in mm within which is to be considered "at the target" for the purpose of terminating movement
#define LINEAR_EPSILON_2               100.0d
//the delta angle within which is to be considered "pointing at the desired orientation" (3 degrees)
#define ANGULAR_EPSILON                  0.05235987755983d

// #define TIMING_MULTIPLIER               2.0d

// unsigned long lastPositionUpdateTimeStamp;

const ZPath* convertToPath(const KMatrix2* start, const Command* command);

ZRoutineController::ZRoutineController(Lighthouse* lighthouse, Zippy* z)
  : lighthouse(lighthouse),
    zippy(z),
    routine(ROUTINE, ROUTINE_POSITION_COUNT)
{
#ifdef ENABLE_SDCARD_LOGGING
  if (SD.begin(CHIP_SELECT_PIN))
    sdCardInserted = true;
  else
    SerialUSB.println("Failed to initialize SD card.");
#endif
}

void ZRoutineController::loop(unsigned long currentTime)
{
  if (!lighthouse->loop(currentTime)) {
    if (lighthouseReady) {
      stopRoutine();
      lighthouseReady = false;
    }
    return;
  }

  if (!lighthouse->recalculate())
    return;

  const KMatrix2* currentPosition = lighthouse->getPosition();
  if (!lighthouseReady) {
    lighthouseReady = true;
    // SerialUSB.println("Starting routine.");
    startRoutine(currentTime, currentPosition);
    // lastPositionUpdateTimeStamp = currentTime;
    return;
  }

  // SerialUSB.println("Executing path.");
  executeCurrentPath(currentTime);

  KMatrix2 relativeTarget(&currentTargetPosition);
  relativeTarget.unconcat(currentPosition);

  //check if we can downgrade our current movement state
  // SerialUSB.println("Checking for move state downgrade.");
  switch (currentMovementState) {
    case MovementMoving:
      if ((!currentCommandPath || !currentCommandPath->updatesPosition()) &&
          relativeTarget.position.getD2() < LINEAR_EPSILON_2)
      {
        if (!currentCommandPath && abs(relativeTarget.orientation.get()) < ANGULAR_EPSILON)
          currentMovementState = MovementStopped;
        else
          currentMovementState = MovementTurning;
      }
      break;

    case MovementTurning:
      if (!currentCommandPath && abs(relativeTarget.orientation.get()) < ANGULAR_EPSILON)
        currentMovementState = MovementStopped;
      break;
  }

  // SerialUSB.println("Executing move.");
  switch (currentMovementState) {
    case MovementMoving:
      zippy->executeMove(lighthouse->getPositionDelta(), &relativeTarget);
      break;

    case MovementTurning:
      zippy->executeTurn(lighthouse->getPositionDelta(), relativeTarget.orientation.get());
      break;

    case MovementStopped:
      zippy->executeStop();
      break;
  }

  zippy->loop(currentTime);

#ifdef ENABLE_SDCARD_LOGGING
  if (logFile) {
    logFile.print(currentTime);

    logFile.print(",");

    logFile.print(currentPosition->position.getX(), 10);
    logFile.print(",");
    logFile.print(currentPosition->position.getY(), 10);
    logFile.print(",");
    logFile.print(currentPosition->orientation.get(), 10);

    logFile.print(",");

    const KMatrix2* velocity = lighthouse->getPositionDelta();
    logFile.print(velocity->position.getX(), 10);
    logFile.print(",");
    logFile.print(velocity->position.getY(), 10);
    logFile.print(",");
    logFile.print(velocity->orientation.get(), 10);

    logFile.print(",");

    logFile.print(currentTargetPosition.position.getX(), 10);
    logFile.print(",");
    logFile.print(currentTargetPosition.position.getY(), 10);
    logFile.print(",");
    logFile.print(currentTargetPosition.orientation.get(), 10);

    logFile.println();
  }
#endif
}

void ZRoutineController::startRoutine(
    unsigned long startTime,
    const KMatrix2* startPosition)
{
#ifdef ENABLE_SDCARD_LOGGING
  if (sdCardInserted) {
    SerialUSB.println("Log file: ");
    String fullLogFilename(LOG_FILENAME);
    fullLogFilename.concat(logNumber);
    fullLogFilename.concat(LOG_FILENAME_EXTENSION);
    SerialUSB.println(fullLogFilename);
    char* logFilename = (char*)fullLogFilename.c_str();
    SD.remove(logFilename);
    logFile = SD.open(logFilename, FILE_WRITE);
    if (logFile) {
      SerialUSB.println("SD card log file opened.");
      logFile.println("t,px,py,po,vx,vy,vo,tx,ty,to");
      logFile.flush();
    }
    else
      SerialUSB.println("Failed to open SD card log file.");
  }
#endif

  currentMovementState = MovementStopped;
  routine.reset();
  currentCommand = routine.getNextCommand();
  currentTargetPosition.set(startPosition);
  planCurrentCommand(startTime);
  zippy->start();
}

void ZRoutineController::planCurrentCommand(unsigned long currentTime)
{
  if (currentCommandPath) {
    //we need to capture the last point on this path segment to use as the starting
    //position to plan the move along the next path segment
    currentCommandPath->interpolate(1.0d, &currentTargetPosition);
    delete currentCommandPath;
  }

  //plan the next path segment
  currentCommandPath = convertToPath(&currentTargetPosition, currentCommand);
  currentCommandStartTime = currentTime;

  //check if we need to upgrade our current movement state
  switch (currentMovementState) {
    case MovementStopped:
      if (currentCommandPath)
        currentMovementState = currentCommandPath->updatesPosition() ? MovementMoving : MovementTurning;
      break;

    case MovementTurning:
      if (currentCommandPath && currentCommandPath->updatesPosition())
        currentMovementState = MovementMoving;
      break;
  }

  if (currentCommand->type == CommandSync)
    lighthouse->clearPreambleFlag();
}

void ZRoutineController::executeCurrentPath(unsigned long currentTime)
{
  unsigned long completionTime = currentCommandCompleted(currentTime);
  if (completionTime) {
    //plan the next path segment
    currentCommand = routine.getNextCommand();
    planCurrentCommand(completionTime);
  }

  if (currentCommandPath) {
#ifdef TIMING_MULTIPLIER
    double interpolatedTime = ((double)(currentTime - currentCommandStartTime)) / (((double)currentCommand->timing) * TIMING_MULTIPLIER);
#else
    double interpolatedTime = ((double)(currentTime - currentCommandStartTime)) / ((double)currentCommand->timing);
#endif
    currentCommandPath->interpolate(interpolatedTime, &currentTargetPosition);
  }
}

unsigned long ZRoutineController::currentCommandCompleted(unsigned long currentTime)
{
  if (currentCommand->type == CommandSync)
    return lighthouse->foundPreamble() ? currentTime : 0;

  unsigned long deltaTime = currentTime - currentCommandStartTime;

#ifdef TIMING_MULTIPLIER
  unsigned long currentCommandDeltaTime = currentCommand->timing * TIMING_MULTIPLIER;
  if (deltaTime > currentCommandDeltaTime) {
    return currentTime - (deltaTime - currentCommandDeltaTime);
  }
#else
  if (deltaTime > currentCommand->timing) {
    return currentTime - (deltaTime - currentCommand->timing);
  }
#endif

  return 0;
}

void ZRoutineController::stopRoutine()
{
#ifdef ENABLE_SDCARD_LOGGING
  if (logFile)
    logFile.close();
  logNumber = (logNumber+1) % LOG_MAX_FILES;
#endif

  // SerialUSB.println("Lighthouse signal lost.");
  zippy->executeStop();

  if (currentCommandPath) {
    delete currentCommandPath;
    currentCommandPath = NULL;
  }
}

const ZPath* convertToPath(const KMatrix2* start, const Command* command)
{
  switch (command->type) {
    case CommandMoveTo:
      return planPath(start, command->params.p1+ZIPPY_OFFSET_X, command->params.p2, command->params.p3);
    case CommandTurnTo:
      return new Turn(start->orientation.get(), subtractAngles(command->params.p1, start->orientation.get()));
    case CommandMove:
      return new Move(start, command->params.p1);
    case CommandArc:
      return new Arc(start, command->params.p1, command->params.p2);
    case CommandTurn:
      return new Turn(start->orientation.get(), command->params.p1);
  }

  return NULL;
}
