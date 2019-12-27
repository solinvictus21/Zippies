
#include "zippies/ZippyControllers.h"
#include "zippies/config/PathingConfig.h"

#define TUNING_TOLERANCE            0.090d
// #define TUNING_DELTA                0.040d
#define TUNING_DELTA                0.100d
#define INITIAL_VELOCITY           60.000d

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#define CHIP_SELECT_PIN            10
#define LOG_FILENAME                 "log"
#define LOG_FILENAME_EXTENSION      ".csv"
#define LOG_MAX_FILES              10
#endif

#ifdef DEBUG_DISPLAY_POSITION
#define DEBUG_DISPLAY_INTERVAL    100
unsigned long previousDebugDisplayTime = 0;
#endif

#define START_OFFSET_Y  -120.0d

Movement moveIntoPlace[]
{
  { MovementType::Turn ,        0.0d               },
  { MovementType::Move ,        0.0d               },
  { MovementType::Turn ,        0.0d               },
};

PathSegment moveIntoPlaceRoutine[]
{
  {     0, 0.05d, 0.05d, 3, moveIntoPlace, 0 },
};

Movement loopsMovements[]
{
  { MovementType::Move ,        10.0d                },
  { MovementType::Arc  ,      -240.0d, -M_PI         },
  { MovementType::Move ,        10.0d                },
  { MovementType::Arc  ,      -240.0d, -M_PI         },
  { MovementType::Move ,        10.0d                },
  { MovementType::Arc  ,       240.0d,  M_PI         },
  { MovementType::Move ,        10.0d                },
  { MovementType::Arc  ,       240.0d,  M_PI         },
  { MovementType::Arc  ,      -120.0d, -M_PI         },
  { MovementType::Arc  ,       120.0d,  2.0d * M_PI  },
  { MovementType::Arc  ,      -120.0d, -M_PI         },
  { MovementType::Arc  ,       120.0d,  M_PI         },
  { MovementType::Arc  ,      -120.0d, -2.0d * M_PI  },
  { MovementType::Arc  ,       120.0d,  M_PI         },
  { MovementType::Arc  ,       -80.0d, -M_PI         },
  { MovementType::Arc  ,        80.0d,  M_PI         },
  { MovementType::Arc  ,       -80.0d, -2.0d * M_PI  },
  { MovementType::Arc  ,        80.0d,  M_PI         },
  { MovementType::Arc  ,       -80.0d, -M_PI         },
  { MovementType::Arc  ,        80.0d,  M_PI         },
  { MovementType::Arc  ,       -80.0d, -M_PI         },
  { MovementType::Arc  ,        80.0d,  2.0d * M_PI  },
  { MovementType::Arc  ,       -80.0d, -M_PI         },
  { MovementType::Arc  ,        80.0d,  M_PI         },
};

PathSegment tuningRoutine[]
{
  { 38000, 0.12d, 0.12d, sizeof(loopsMovements) / sizeof(Movement), loopsMovements, 0 },
};

PIDTuningController::PIDTuningController(SensorFusor* s, Zippy* z)
  : sensors(s),
    zippy(z),
    routineController(z)
{
  // setupTuning(TuningVariable::Proportional);
  // setupTuning(TuningVariable::Integral);
  setupTuning(TuningVariable::Derivative);
}

void PIDTuningController::start(unsigned long currentTime)
{
  //setup our initial move into the starting position for testing
  currentTestingState = TestingState::MovingIntoPlace;

  //ensure that the zippy isn't going to capture errors during the initial move
  zippy->stopErrorCapture();

  //calculate our path to the initial starting position
  const KMatrix2* currentPosition = sensors->getPosition();
  double deltaX = PATHING_OFFSET_X - currentPosition->position.getX();
  double deltaY = PATHING_OFFSET_Y - currentPosition->position.getY();
  double deltaO = atan2(deltaX, deltaY);
  double distanceToStartingPoint = sqrt(sq(deltaX) + sq(deltaY));

  //setup the three moves to arrive at the starting position
  //turn toward the starting position
  moveIntoPlace[0].params.p1 = subtractAngles(deltaO, currentPosition->orientation.get());
  //follow a straight light toward the starting position
  moveIntoPlace[1].params.p1 = distanceToStartingPoint;
  //face forward
  moveIntoPlace[2].params.p1 = -deltaO;

  //calculate the timing for the three moves based on the distance we'll need to move
  moveIntoPlaceRoutine[0].timing = 1000.0d * (distanceToStartingPoint / INITIAL_VELOCITY);

  //provide the move-into-place routine to our routing controller
  routineController.setRoutine(moveIntoPlaceRoutine, (int)(sizeof(moveIntoPlaceRoutine) / sizeof(PathSegment)));
  routineController.setAnchorPosition(currentPosition);

  //start the routine controller
  routineController.start(currentTime);

#ifdef DEBUG_DISPLAY_POSITION
  previousDebugDisplayTime = currentTime;
#endif
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

  zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
}

void PIDTuningController::loop(unsigned long currentTime)
{
  switch (currentTestingState) {
    case TestingState::MovingIntoPlace:
      routineController.loop(currentTime);
      if (routineController.isRoutineCompleted())
        currentTestingState = TestingState::WaitingForFullStop;
      break;

    case TestingState::WaitingForFullStop:
      if (zippy->isStopped()) {
        sensors->clearPreambleFlag();
        currentTestingState = TestingState::SyncingWithPreamble;
      }
      break;

    case TestingState::SyncingWithPreamble:
      if (sensors->foundPreamble()) {
#ifdef ENABLE_SDCARD_LOGGING
        openLogFile(currentTime);
#endif
        zippy->startErrorCapture();
        // routineController.setRoutine(tuningRoutine, (int)(sizeof(tuningRoutine) / sizeof(Command)), 1);
        routineController.setRoutine(tuningRoutine, (int)(sizeof(tuningRoutine) / sizeof(PathSegment)));
        routineController.setAnchorPosition(PATHING_OFFSET_X, PATHING_OFFSET_Y, 0.0d);
        routineController.start(currentTime);
        currentTestingState = TestingState::Testing;
      }
      break;

    case TestingState::Testing:
#ifdef ENABLE_SDCARD_LOGGING
      writeLogData(currentTime);
#endif
      routineController.loop(currentTime);
      if (routineController.isRoutineCompleted()) {
        //evaluate the performance of the current PID values
        zippy->stopErrorCapture();
        evaluateTuning();

        //move back into place to prepare to restart the test
        sensors->clearPreambleFlag();
        currentTestingState = TestingState::SyncingWithPreamble;
      }
      break;
  }

#ifdef DEBUG_DISPLAY_POSITION
  if (currentTime - previousDebugDisplayTime >= DEBUG_DISPLAY_INTERVAL) {
    previousDebugDisplayTime += DEBUG_DISPLAY_INTERVAL;

    ZippyFace* face = zippy->getFace();
    face->clearScreen();
    uint8_t fontHeight = face->getFontHeight();
    uint8_t currentRow = fontHeight;
    const KMatrix2* currentPosition = sensors->getPosition();
    face->displayLabelAndData(
        0,
        currentRow,
        "X",
        currentPosition->position.getX(),
        2);
    currentRow += fontHeight;
    face->displayLabelAndData(
        0,
        currentRow,
        "Y",
        currentPosition->position.getY(),
        2);
    currentRow += fontHeight;
    face->displayLabelAndData(
        0,
        currentRow,
        "O",
        currentPosition->orientation.get(),
        2);
  }
#endif
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
  currentTestRun.error = zippy->getStandardDeviation();
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

  ZippyFace* face = zippy->getFace();
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

  zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
}

void PIDTuningController::stop()
{
#ifdef ENABLE_SDCARD_LOGGING
  closeLogFile();
#endif
  routineController.stop();
  zippy->stopErrorCapture();
}

void PIDTuningController::displayTestData(
    uint8_t y,
    const char* pl, double p,
    const char* il, double i,
    const char* dl, double d)
{
  ZippyFace* face = zippy->getFace();
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
