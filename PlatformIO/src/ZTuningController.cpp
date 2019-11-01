
#include "ZTuningController.h"

// #define DEBUG_DISPLAY_POSITION      1

#define KP_START                  120.000d
#define KI_START                    0.000d
#define KD_START                    5.000d
#define TUNING_TOLERANCE            0.200d
#define INITIAL_VELOCITY          120.000d

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

Command moveIntoPlaceRoutine[]
{
  {             4000, CommandMoveTo,  ZIPPY_OFFSET_X,  -120.0d,    0.0d },
};

Command tuningRoutine[]
{
  // {            10000, CommandPause                                      },
// /*
  //do a figure eight, first left then right
  {             2800, CommandArc,            -250.0d,      -M_PI_2      },
  {             2300, CommandArc,            -250.0d,      -M_PI_2      },
  {             1800, CommandArc,            -250.0d,      -M_PI_2      },
  {             1600, CommandArc,            -250.0d,      -M_PI_2      },
  {             1400, CommandArc,             250.0d,       M_PI_2      },
  {             1300, CommandArc,             250.0d,       M_PI_2      },
  {             1200, CommandArc,             250.0d,       M_PI_2      },
  {             1100, CommandArc,             250.0d,       M_PI_2      },
// */
/*
  //2nd
  {             1000, CommandArc,            -250.0d,      -M_PI_2      },
  {              930, CommandArc,            -250.0d,      -M_PI_2      },
  {              870, CommandArc,            -250.0d,      -M_PI_2      },
  {              820, CommandArc,            -250.0d,      -M_PI_2      },
  {              780, CommandArc,             250.0d,       M_PI_2      },
  {              750, CommandArc,             250.0d,       M_PI_2      },
  {              730, CommandArc,             250.0d,       M_PI_2      },
  {              720, CommandArc,             250.0d,       M_PI_2      },
// */
/*
  //3rd
  {              715, CommandArc,            -250.0d,      -M_PI_2      },
  {              710, CommandArc,            -250.0d,      -M_PI_2      },
  {              705, CommandArc,            -250.0d,      -M_PI_2      },
  {              700, CommandArc,            -250.0d,      -M_PI_2      },
  {              700, CommandArc,             250.0d,       M_PI_2      },
  {              705, CommandArc,             250.0d,       M_PI_2      },
  {              710, CommandArc,             250.0d,       M_PI_2      },
  {              715, CommandArc,             250.0d,       M_PI_2      },
// */
/*
  //4th
  {              720, CommandArc,            -250.0d,      -M_PI_2      },
  {              730, CommandArc,            -250.0d,      -M_PI_2      },
  {              750, CommandArc,            -250.0d,      -M_PI_2      },
  {              780, CommandArc,            -250.0d,      -M_PI_2      },
  {              820, CommandArc,             250.0d,       M_PI_2      },
  {              870, CommandArc,             250.0d,       M_PI_2      },
  {              930, CommandArc,             250.0d,       M_PI_2      },
  {             1000, CommandArc,             250.0d,       M_PI_2      },
// */
// /*
  //5th
  {             1100, CommandArc,            -250.0d,      -M_PI_2      },
  {             1200, CommandArc,            -250.0d,      -M_PI_2      },
  {             1400, CommandArc,            -250.0d,      -M_PI_2      },
  {             1600, CommandArc,            -250.0d,      -M_PI_2      },
  {             1900, CommandArc,             250.0d,       M_PI_2      },
  {             2100, CommandArc,             250.0d,       M_PI_2      },
  {             2500, CommandArc,             250.0d,       M_PI_2      },
  {             2800, CommandArc,             250.0d,       M_PI_2      },
// */
};

ZTuningController::ZTuningController(SensorFusor* s, Zippy* z)
  : sensors(s),
    zippy(z),
    routineController(z)
{
  // setupTuning(TuningVariable::Proportional);
  // setupTuning(TuningVariable::Integral);
  setupTuning(TuningVariable::Derivative);
}

void ZTuningController::start(unsigned long currentTime)
{
  currentTestingState = TestingState::MovingIntoPlace;
  zippy->stopErrorCapture();
  const KMatrix2* currentPosition = sensors->getPosition();
  double distanceToStartingPoint = sqrt(
      sq(currentPosition->position.getX() - moveIntoPlaceRoutine[0].params.p1) +
      sq(currentPosition->position.getY() - moveIntoPlaceRoutine[0].params.p2));
  moveIntoPlaceRoutine[0].timing = 1000.0d * (distanceToStartingPoint / INITIAL_VELOCITY);
  routineController.setRoutine(moveIntoPlaceRoutine, (int)(sizeof(moveIntoPlaceRoutine) / sizeof(Command)), 1);
  routineController.setStartPosition(currentPosition);
  routineController.start(currentTime);

#ifdef DEBUG_DISPLAY_POSITION
  previousDebugDisplayTime = currentTime;
#endif
}

void ZTuningController::setupTuning(TuningVariable tuningVariable)
{
  currentTestRun.Kp = KP_START;
  currentTestRun.Ki = KI_START;
  currentTestRun.Kd = KD_START;
  bestTestRun.error = 1000000.0d;
  bestTestRun.range = 1000000.0d;

  currentTuningVariable = tuningVariable;
  switch (currentTuningVariable) {
    default:
    case TuningVariable::Proportional:
      currentTestValue = &currentTestRun.Kp;
      currentBestTestValue = &bestTestRun.Kp;
      break;
    case TuningVariable::Integral:
      currentTestValue = &currentTestRun.Ki;
      currentBestTestValue = &bestTestRun.Ki;
      break;
    case TuningVariable::Derivative:
      currentTestValue = &currentTestRun.Kd;
      currentBestTestValue = &bestTestRun.Kd;
      break;
  }
  currentTestIncrement = 0.1d * (*currentTestValue);

  zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
}

void ZTuningController::loop(unsigned long currentTime)
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
        routineController.setRoutine(tuningRoutine, (int)(sizeof(tuningRoutine) / sizeof(Command)), 1);
        routineController.start(currentTime);
        currentTestingState = TestingState::Testing;
      }
      break;

    case TestingState::Testing:
      routineController.loop(currentTime);
#ifdef ENABLE_SDCARD_LOGGING
      writeLogData(currentTime);
#endif
      if (routineController.isRoutineCompleted()) {
        //evaluate the performance of the current PID values
        zippy->stopErrorCapture();
        evaluateTuning();

        //move back into place to prepare to restart the test
        sensors->clearPreambleFlag();
        currentTestingState = TestingState::WaitingForFullStop;
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

void ZTuningController::evaluateTuning()
{
  currentTestRun.error = zippy->getErrorAverage();
  currentTestRun.range = zippy->getErrorRange();
  double deltaError = abs((currentTestRun.range - bestTestRun.range) / bestTestRun.range);
  if (deltaError > TUNING_TOLERANCE) {
    if (abs(currentTestRun.range) < abs(bestTestRun.range)) {
      //movement accuracy has improved, so just keep moving the test value in the same direction
      memcpy(&bestTestRun, &currentTestRun, sizeof(AutoTuneTest));
      // (*currentBestTestValue) = (*currentTestValue);
      // bestTestRun.error = currentTestRun.error;
      // bestTestRun.range = currentTestRun.range;
    }
    else if (((*currentTestValue) - (*currentBestTestValue)) * currentTestIncrement > 0.0d) {
      //movement accuracy has become worse; if the increment is moving away from the best measured value, then move
      //the test value in the opposite direction
      currentTestIncrement = -currentTestIncrement*2.0d/3.0d;
    }
  }

  ZippyFace* face = zippy->getFace();
  face->clearScreen();
  face->displayLabelAndData(SCREEN_WIDTH_PIXELS_2, 0, "P", (*currentTestValue), 2);
  face->displayLabelAndData(SCREEN_WIDTH_PIXELS_2, face->getFontHeight(), "PE", currentTestRun.error, 3);
  face->displayLabelAndData(SCREEN_WIDTH_PIXELS_2, 2 * face->getFontHeight(), "PR", currentTestRun.range, 3);

  (*currentTestValue) += currentTestIncrement;
  zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);

  face->displayLabelAndData(0, 0, "C", (*currentTestValue), 2);
  displayTestData(0, bestTestRun.Kp, bestTestRun.Ki, bestTestRun.Kd, bestTestRun.error, bestTestRun.range);
}

void ZTuningController::stop()
{
#ifdef ENABLE_SDCARD_LOGGING
  closeLogFile();
#endif
  routineController.stop();
  zippy->stopErrorCapture();
}

void ZTuningController::displayTestData(uint8_t x, double p, double i, double d, double e, double r)
{
  ZippyFace* face = zippy->getFace();
  uint8_t fontHeight = face->getFontHeight();
  uint8_t currentRow = fontHeight;
  face->displayLabelAndData(
      x,
      currentRow,
      "P",
      p,
      2);
  currentRow += fontHeight;
  face->displayLabelAndData(
      x,
      currentRow,
      "I",
      i,
      2);
  currentRow += fontHeight;
  face->displayLabelAndData(
      x,
      currentRow,
      "D",
      d,
      2);
  currentRow += fontHeight;
  face->displayLabelAndData(
      x,
      currentRow,
      "E",
      e,
      3);
  currentRow += fontHeight;
  face->displayLabelAndData(
      x,
      currentRow,
      "R",
      r,
      3);
}

#ifdef ENABLE_SDCARD_LOGGING
void ZTuningController::openLogFile(unsigned long currentTime)
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

void ZTuningController::writeLogData(unsigned long currentTime)
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

void ZTuningController::closeLogFile()
{
  if (!logFile)
    return;

  logFile.close();
  logNumber = (logNumber+1) % LOG_MAX_FILES;
  // SerialUSB.println("SD card log file closed.");
}
#endif //SD_CARD_LOGGING
