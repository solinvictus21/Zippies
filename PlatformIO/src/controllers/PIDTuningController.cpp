
#include "zippies/ZippyControllers.h"
#include "zippies/ZippyRoutine.h"

#define TUNING_TOLERANCE                 0.090d
#define TUNING_DELTA                     0.100d

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#define CHIP_SELECT_PIN            10
#define LOG_FILENAME                 "log"
#define LOG_FILENAME_EXTENSION      ".csv"
#define LOG_MAX_FILES              10
#endif

#define START_OFFSET_Y  -120.0d

PIDTuningController::PIDTuningController()
{
  setupTuning(TuningVariable::Proportional);
  // setupTuning(TuningVariable::Integral);
  // setupTuning(TuningVariable::Derivative);
}

void PIDTuningController::start(unsigned long currentTime)
{
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

  // pathFollowingController.getZippy()->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
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

  /*
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

  zippy->setTunings(currentTestRun.Kp, currentTestRun.Ki, currentTestRun.Kd);
  */
}

void PIDTuningController::stop()
{
#ifdef ENABLE_SDCARD_LOGGING
  closeLogFile();
#endif
}

void PIDTuningController::displayTestData(
    uint8_t y,
    const char* pl, double p,
    const char* il, double i,
    const char* dl, double d)
{
  /*
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
      */
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

  const ZMatrix2* velocity = lighthouse->getPositionDelta();
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
  const ZVector3* directionToLighthouse = sensors->getLeftSensor()->getDirectionFromLighthouse();
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
