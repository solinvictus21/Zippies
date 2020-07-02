
#include "zippies/ZippyControllers.h"

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>

#define CHIP_SELECT_PIN        10
#define LOG_FILENAME           "direct"
#define LOG_FILENAME_EXTENSION ".csv"
#define LOG_MAX_FILES          10
bool sdCardInitialized = false;
File directLogFile;
int directLogNumber = 0;
#endif

typedef struct _DirectMove
{
  unsigned long deltaTime;
  double leftWheelPower;
  double rightWheelPower;
} DirectMove;

DirectMove movements[] = {
  // {   8000,   3000.0d,    300.0d },
  {   8000,  10000.0d,  10000.0d },
  {   5000,      0.0d,      0.0d },
};
int movementCount = (int)(sizeof(movements) / sizeof(DirectMove));

DirectController::DirectController()
{
#ifdef ENABLE_SDCARD_LOGGING
  if (SD.begin(CHIP_SELECT_PIN))
    sdCardInitialized = true;
  else
    SerialUSB.println("Failed to initialize SD card.");
#endif
}

void DirectController::loop(unsigned long currentTime)
{
  if (!sensors.loop(currentTime)) {
    if (lighthouseReady) {
      previousPositionTimeStamp = 0;
      stopController(currentTime);
      lighthouseReady = false;
    }
    return;
  }

  //check if we have a position update
  if (sensors.getPositionTimeStamp() <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = sensors.getPositionTimeStamp();

  if (!lighthouseReady) {
    lighthouseReady = true;
    startController(currentTime);
    return;
  }

  loopController(currentTime);
}

void DirectController::startController(unsigned long currentTime)
{
#ifdef ENABLE_SDCARD_LOGGING
  if (sdCardInitialized) {
    String fullLogFilename(LOG_FILENAME);
    fullLogFilename.concat(directLogNumber);
    fullLogFilename.concat(LOG_FILENAME_EXTENSION);

    // SerialUSB.print("Log file: ");
    // SerialUSB.println(fullLogFilename);

    char* directLogFilename = (char*)fullLogFilename.c_str();
    SD.remove(directLogFilename);
    directLogFile = SD.open(directLogFilename, FILE_WRITE);
    if (directLogFile) {
      // SerialUSB.println("SD card log file opened.");
      directLogFile.println("t,px,py,po,vx,vy,vo");
      directLogFile.flush();
    }
    else
      SerialUSB.println("Failed to open SD card log file.");
  }
#endif

  currentMoveIndex = 0;
  startNextMove(currentTime);
}

void DirectController::startNextMove(unsigned long currentTime)
{
  currentMoveStartTime = currentTime;
  currentMoveDeltaTime = movements[currentMoveIndex].deltaTime;
  // SerialUSB.print("Setting motors: ");
  // SerialUSB.print(movements[currentMoveIndex].leftWheelPower, 0);
  // SerialUSB.print(", ");
  // SerialUSB.println(movements[currentMoveIndex].rightWheelPower, 0);
  motors.setMotors(movements[currentMoveIndex].leftWheelPower, movements[currentMoveIndex].rightWheelPower);
  currentMoveIndex++;
  if (currentMoveIndex == movementCount)
    currentMoveIndex = 0;
}

void DirectController::loopController(unsigned long currentTime)
{
#ifdef ENABLE_SDCARD_LOGGING
  if (directLogFile) {
    directLogFile.print(currentTime);

    directLogFile.print(",");

    const ZMatrix2* currentPosition = sensors.getPosition();
    directLogFile.print(currentPosition->position.getX(), 10);
    directLogFile.print(",");
    directLogFile.print(currentPosition->position.getY(), 10);
    directLogFile.print(",");
    directLogFile.print(currentPosition->orientation.get(), 10);

    directLogFile.print(",");

    const ZMatrix2* velocity = sensors.getPositionDelta();
    directLogFile.print(velocity->position.getX(), 10);
    directLogFile.print(",");
    directLogFile.print(velocity->position.getY(), 10);
    directLogFile.print(",");
    directLogFile.print(velocity->orientation.get(), 10);

    directLogFile.println();
  }
#endif

  if (currentTime - currentMoveStartTime <= currentMoveDeltaTime)
    return;

  currentTime -= currentMoveDeltaTime;
  startNextMove(currentTime);
}

void DirectController::stopController(unsigned long currentTime)
{
#ifdef ENABLE_SDCARD_LOGGING
  if (directLogFile)
    directLogFile.close();
  directLogNumber = (directLogNumber+1) % LOG_MAX_FILES;
#endif

  motors.stopMotors();
}
