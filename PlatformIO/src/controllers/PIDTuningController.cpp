
#include "zippies/ZippyControllers.h"
#include "zippies/config/PathingConfig.h"
#include "zippies/ZippyPaths.h"

#define TUNING_TOLERANCE                 0.090d
#define TUNING_DELTA                     0.100d
#define INITIAL_MOVE_VELOCITY           80.000d
#define INITIAL_MOVE_TIMING          12000

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#define CHIP_SELECT_PIN            10
#define LOG_FILENAME                 "log"
#define LOG_FILENAME_EXTENSION      ".csv"
#define LOG_MAX_FILES              10
#endif

#define START_OFFSET_Y  -120.0d

PathDefinition moveIntoPlace[]
{
  { PathDefinitionType::Arc ,        0.0d               },
  { PathDefinitionType::Arc ,        0.0d               },
};

RoutineDefinition moveIntoPlaceRoutine[]
{
  //bi-arc move into place
  {     0, 0.05d, 0.05d, sizeof(moveIntoPlace) / sizeof(PathDefinition), moveIntoPlace, 0 },
  //initial pause
  {     0, 0.00d, 0.00d, 0,          NULL, 0 },
};

PathDefinition loopsPathDefinition[]
{
  { PathDefinitionType::Move ,        10.0d                },
  { PathDefinitionType::Arc  ,      -240.0d, -M_PI         },
  { PathDefinitionType::Move ,        10.0d                },
  { PathDefinitionType::Arc  ,      -240.0d, -M_PI         },
  { PathDefinitionType::Move ,        10.0d                },
  { PathDefinitionType::Arc  ,       240.0d,  M_PI         },
  { PathDefinitionType::Move ,        10.0d                },
  { PathDefinitionType::Arc  ,       240.0d,  M_PI         },
  { PathDefinitionType::Arc  ,      -120.0d, -M_PI         },
  { PathDefinitionType::Arc  ,       120.0d,  2.0d * M_PI  },
  { PathDefinitionType::Arc  ,      -120.0d, -M_PI         },
  { PathDefinitionType::Arc  ,       120.0d,  M_PI         },
  { PathDefinitionType::Arc  ,      -120.0d, -2.0d * M_PI  },
  { PathDefinitionType::Arc  ,       120.0d,  M_PI         },
  { PathDefinitionType::Arc  ,       -80.0d, -M_PI         },
  { PathDefinitionType::Arc  ,        80.0d,  M_PI         },
  { PathDefinitionType::Arc  ,       -80.0d, -2.0d * M_PI  },
  { PathDefinitionType::Arc  ,        80.0d,  M_PI         },
  { PathDefinitionType::Arc  ,       -80.0d, -M_PI         },
  { PathDefinitionType::Arc  ,        80.0d,  M_PI         },
  { PathDefinitionType::Arc  ,       -80.0d, -M_PI         },
  { PathDefinitionType::Arc  ,        80.0d,  2.0d * M_PI  },
  { PathDefinitionType::Arc  ,       -80.0d, -M_PI         },
  { PathDefinitionType::Arc  ,        80.0d,  M_PI         },
};

RoutineDefinition tuningRoutineDefinition[]
{
  { 40000, 0.12d, 0.12d, sizeof(loopsPathDefinition) / sizeof(PathDefinition), loopsPathDefinition, 0 },
};

PIDTuningController::PIDTuningController(SensorFusor* s)
  : sensors(s),
    routine(),
    pathFollowingController()
{
  setupTuning(TuningVariable::Proportional);
  // setupTuning(TuningVariable::Integral);
  // setupTuning(TuningVariable::Derivative);
}

void createRelativePathDefinition(const KVector2* relativeMovement, PathDefinition* pathDefinition)
{
  double direction = relativeMovement->atan();
  if (direction == 0.0d) {
    pathDefinition->type = PathDefinitionType::Move;
    pathDefinition->params.p1 = relativeMovement->getY();
  }
  else if (relativeMovement->getD2() == 0.0d) {
    pathDefinition->type = PathDefinitionType::Turn;
    pathDefinition->params.p1 = direction;
  }
  else {
    pathDefinition->type = PathDefinitionType::Arc;
    pathDefinition->params.p1 = relativeMovement->getD() / (2.0d * sin(relativeMovement->atan2()));
    pathDefinition->params.p2 = 2.0d * direction;
  }
}

void PIDTuningController::start(unsigned long currentTime)
{
  sensors->syncWithPreamble();
  currentTestingState = TestingState::PreSyncingWithPreamble;
}

void PIDTuningController::planMoveIntoPlace(const KMatrix2* fromPosition)
{
  //calculate our path to the initial starting position
  KMatrix2 movement2(PATHING_OFFSET_X, PATHING_OFFSET_Y, 0.0d);
  movement2.unconcat(fromPosition);
  KMatrix2 movement1;
  calculateRelativeBiArcKnot(&movement2, &movement1);
  movement2.unconcat(&movement1);
  createRelativePathDefinition(&movement1.position, &moveIntoPlace[0]);
  createRelativePathDefinition(&movement2.position, &moveIntoPlace[1]);

  double pathDistance =
      getPathSegmentLength(&moveIntoPlace[0]) +
      getPathSegmentLength(&moveIntoPlace[1]);
  moveIntoPlaceRoutine[0].timing = min(1000.0d * (pathDistance / INITIAL_MOVE_VELOCITY), INITIAL_MOVE_TIMING);
  moveIntoPlaceRoutine[1].timing = INITIAL_MOVE_TIMING - moveIntoPlaceRoutine[0].timing;
}

void PIDTuningController::startMoveIntoPlace(unsigned long currentTime)
{
  //calculate our path to the initial starting position
  const KMatrix2* currentPosition = sensors->getPosition();
  planMoveIntoPlace(currentPosition);

  //provide the move-into-place routine to our routing controller
  previousTargetPosition.set(currentPosition);
  routine.setRoutineSegments(
      &previousTargetPosition,
      moveIntoPlaceRoutine,
      (int)(sizeof(moveIntoPlaceRoutine) / sizeof(RoutineDefinition)));

  //start the routine controller
  routine.start(currentTime);
}

void PIDTuningController::setupTuning(TuningVariable tuningVariable)
{
  currentTuningVariable = tuningVariable;
  switch (currentTuningVariable) {
    default:
    case TuningVariable::Proportional:
      currentTestValue = &currentTestRun.Kp;
      bestTestValue = &bestTestRun.Kp;
      break;
    case TuningVariable::Integral:
      currentTestValue = &currentTestRun.Ki;
      bestTestValue = &bestTestRun.Ki;
      break;
    case TuningVariable::Derivative:
      currentTestValue = &currentTestRun.Kd;
      bestTestValue = &bestTestRun.Kd;
      break;
  }
  currentTestIncrement = (*currentTestValue) * TUNING_DELTA;

  pathFollowingController.getZippy()->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
}

void PIDTuningController::captureError()
{
  //TODO: capture error
  /*
  //only capture error statistics while moving
  if (errorCaptureEnabled && currentMovementState == MovementState::Moving) {
    //calculate error based on how close we came to our target position
    double error = sqrt(
        sq(currentPosition.position.getX() - targetPosition.position.getX()) +
        sq(currentPosition.position.getY() - targetPosition.position.getY()));
    statisticsAccumulator.accumulate(error);
  }
  */
}

void PIDTuningController::loop(unsigned long currentTime)
{
  switch (currentTestingState) {

    case TestingState::PreSyncingWithPreamble:
      if (sensors->foundPreamble()) {
        // SerialUSB.println("Found preamble. Moving into position.");
        startMoveIntoPlace(currentTime);
        currentTestingState = TestingState::MovingIntoPlace;
      }
      break;

    case TestingState::MovingIntoPlace:
      routine.loop(currentTime);
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (routine.isRoutineCompleted()) {
        // SerialUSB.println("In position. Starting test.");
        statisticsAccumulator.reset();
        previousTargetPosition.set(PATHING_OFFSET_X, PATHING_OFFSET_Y, 0.0d);
        routine.setRoutineSegments(&previousTargetPosition, tuningRoutineDefinition, (int)(sizeof(tuningRoutineDefinition) / sizeof(RoutineDefinition)));
        routine.start(currentTime);
        currentTestingState = TestingState::Testing;
      }
      break;

    case TestingState::Testing:
#ifdef ENABLE_SDCARD_LOGGING
      writeLogData(currentTime);
#endif
      captureError();
      routine.loop(currentTime);
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (routine.isRoutineCompleted()) {
        //evaluate the performance of the current PID values
        // evaluateTuning();

        //move back into place to prepare to restart the test
        sensors->syncWithPreamble();
        currentTestingState = TestingState::PostSyncingWithPreamble;
      }
      break;

    case TestingState::PostSyncingWithPreamble:
      pathFollowingController.followPath(
          sensors->getPosition(),
          routine.getTargetPosition(),
          routine.getTargetMovementState());

      if (sensors->foundPreamble()) {
        statisticsAccumulator.reset();
        previousTargetPosition.set(PATHING_OFFSET_X, PATHING_OFFSET_Y, 0.0d);
        routine.start(currentTime);
        currentTestingState = TestingState::Testing;
      }
      break;
  }
}

bool willMoveAwayFromIdeal(double ideal, double previous, double increment)
{
  if (previous > ideal)
    return increment > 0.0d;
  else
    return increment < 0.0d;
}

void PIDTuningController::evaluateTuning()
{
  // currentTestRun.error = zippy->getErrorAverage();
  currentTestRun.error = statisticsAccumulator.getStandardDeviation();
  // currentTestRun.range = zippy->getErrorRange();
  // double deltaError = abs((currentTestRun.range - bestTestRun.range) / bestTestRun.range);
  double deltaErrorPercentage = (currentTestRun.error - bestTestRun.error) / bestTestRun.error;
  if (abs(deltaErrorPercentage) > TUNING_TOLERANCE) {
    if (abs(currentTestRun.error) < abs(bestTestRun.error)) {
      //movement accuracy has improved, so just keep moving the test value in the same direction
      memcpy(&bestTestRun, &currentTestRun, sizeof(AutoTuneTest));
    }
    else if (willMoveAwayFromIdeal((*bestTestValue), (*currentTestValue), currentTestIncrement))
      currentTestIncrement = -currentTestIncrement * 0.7d;
  }

  ZippyFace* face = pathFollowingController.getZippy()->getFace();
  face->clearScreen();
  displayTestData(
      0,
      "PP", currentTestRun.Kp, "PI", currentTestRun.Ki,
      "PD", currentTestRun.Kd);
  face->displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, face->getFontHeight(),
      "PE", currentTestRun.error, 2);

  (*currentTestValue) += currentTestIncrement;

  displayTestData(
      2 * face->getFontHeight(),
      "CP", currentTestRun.Kp, "CI", currentTestRun.Ki,
      "CD", currentTestRun.Kd);
  face->displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, 3 * face->getFontHeight(),
      "I", currentTestIncrement, 2);
  displayTestData(
      4 * face->getFontHeight(),
      "BP", bestTestRun.Kp, "BI", bestTestRun.Ki,
      "BD", bestTestRun.Kd);
  face->displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, 5 * face->getFontHeight(),
      "CE", bestTestRun.error, 2);

  // zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
}

void PIDTuningController::stop()
{
#ifdef ENABLE_SDCARD_LOGGING
  closeLogFile();
#endif
  pathFollowingController.stop();
}

void PIDTuningController::displayTestData(
    uint8_t y,
    const char* pl, double p,
    const char* il, double i,
    const char* dl, double d)
{
  ZippyFace* face = pathFollowingController.getZippy()->getFace();
  uint8_t fontHeight = face->getFontHeight();
  face->displayLabelAndData(
      0,
      y,
      pl,
      p,
      2);
  face->displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2,
      y,
      il,
      i,
      2);
  y += fontHeight;
  face->displayLabelAndData(
      0,
      y,
      dl,
      d,
      2);
}

#ifdef ENABLE_SDCARD_LOGGING
void PIDTuningController::openLogFile(unsigned long currentTime)
{
  if (!SD.begin(CHIP_SELECT_PIN)) {
    SerialUSB.println("Failed to initialize SD card.");
    return;
  }

  String fullLogFilename(LOG_FILENAME);
  fullLogFilename.concat(logNumber);
  fullLogFilename.concat(LOG_FILENAME_EXTENSION);
  // SerialUSB.print("Log file: ");
  // SerialUSB.println(fullLogFilename);
  char* logFilename = (char*)fullLogFilename.c_str();
  SD.remove(logFilename);
  logFile = SD.open(logFilename, FILE_WRITE);
  if (logFile) {
    // SerialUSB.println("SD card log file opened.");
    // logFile.println("t,px,py,po,vx,vy,vo,tx,ty,to");
    logFile.println("d0x,d0y,d0z,d1x,d1y,d1z,la0x,la0z,la1x,la1z");
    logFile.flush();
  }
  else
    SerialUSB.println("Failed to open SD card log file.");
}

void PIDTuningController::writeLogData(unsigned long currentTime)
{
  if (!logFile)
    return;

  /*
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
  */
  const KVector3* directionToLighthouse = sensors->getLeftSensor()->getDirectionFromLighthouse();
  logFile.print(directionToLighthouse->getX(), 10);
  logFile.print(",");
  logFile.print(directionToLighthouse->getY(), 10);
  logFile.print(",");
  logFile.print(directionToLighthouse->getZ(), 10);

  logFile.print(",");

  directionToLighthouse = sensors->getRightSensor()->getDirectionFromLighthouse();
  logFile.print(directionToLighthouse->getX(), 10);
  logFile.print(",");
  logFile.print(directionToLighthouse->getY(), 10);
  logFile.print(",");
  logFile.print(directionToLighthouse->getZ(), 10);

  logFile.print(",");

  logFile.print(sensors->getLeftSensor()->getDetectedHitX(), 10);
  logFile.print(",");
  logFile.print(sensors->getLeftSensor()->getDetectedHitZ(), 10);
  logFile.print(",");
  logFile.print(sensors->getRightSensor()->getDetectedHitX(), 10);
  logFile.print(",");
  logFile.print(sensors->getRightSensor()->getDetectedHitZ(), 10);

  logFile.println();
}

void PIDTuningController::closeLogFile()
{
  if (!logFile)
    return;

  logFile.close();
  logNumber = (logNumber+1) % LOG_MAX_FILES;
  // SerialUSB.println("SD card log file closed.");
}
#endif //SD_CARD_LOGGING
