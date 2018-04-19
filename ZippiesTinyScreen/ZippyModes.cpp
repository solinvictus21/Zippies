
#include "ZippyModes.h"
#include "Bluetooth.h"
#include "ZippyFace.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

#define AUTODRIVE_ENABLED

#define AUTODRIVE_MISSING_POSITION_TIMEOUT    1000
#define AUTODRIVE_CORRECTION_INTERVAL_MS     50000
#define AUTODRIVE_REAR_POSITION               -800.0d
#define AUTODRIVE_FRONT_POSITION                 0.0d
#define AUTODRIVE_LEFT_POSITION               -600.0d
#define AUTODRIVE_RIGHT_POSITION               600.0d

#define ZIPPY_COMMAND_COUNT 2

extern ZippyFace face;
extern Bluetooth bluetooth;
extern Lighthouse lighthouse;
extern MotorDriver motors;

AutoDriveMode::AutoDriveMode()
  : moving(false),
    lostPositionTimestamp(0),
    lastCorrectionTime(0),
    currentCommand(0)
{
  //rectangle
//  pathPoints[0].set(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_REAR_POSITION);
//  pathPoints[1].set(AUTODRIVE_LEFT_POSITION, AUTODRIVE_REAR_POSITION);
//  pathPoints[2].set(AUTODRIVE_LEFT_POSITION, AUTODRIVE_FRONT_POSITION);
//  pathPoints[3].set(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_FRONT_POSITION);

  //circle
  pathPoints[0].set(    0.0d,  300.0d);
  pathPoints[1].set( -150.0d,  220.0d);
  pathPoints[2].set( -500.0d,   50.0d);
  pathPoints[3].set( -600.0d, -250.0d);
  pathPoints[4].set( -500.0d, -550.0d);
  pathPoints[5].set( -150.0d, -720.0d);
  pathPoints[6].set(    0.0d, -800.0d);
  pathPoints[7].set(  150.0d, -720.0d);
  pathPoints[8].set(  500.0d, -550.0d);
  pathPoints[9].set(  600.0d, -250.0d);
  pathPoints[10].set( 500.0d,   50.0d);
  pathPoints[11].set( 150.0d,  220.0d);
  pathPoints[12].set(    0.0d, 300.0d);
  
  commands = new ZippyCommand*[ZIPPY_COMMAND_COUNT];
  commands[0] = new Pause(3.0d);
  commands[1] = new FollowPath(pathPoints, PATH_POINT_COUNT, AUTODRIVE_CORRECTION_INTERVAL_MS/1000);
}

void AutoDriveMode::loop()
{
  if (currentCommand >= ZIPPY_COMMAND_COUNT)
    return;

//  static int skipCount = 0;
  unsigned long currentTime = micros();
  if (!lighthouse.hasLighthouseSignal()) {
    if (!moving)
      return;
      
    //the sensors do not have the latest robot position; wait until they do; if we're currently in motion, setup a
    //timeout to stop moving if we go too long without the position information
    if (!lostPositionTimestamp)
      lostPositionTimestamp = currentTime;
    else if (currentTime - lostPositionTimestamp >= AUTODRIVE_MISSING_POSITION_TIMEOUT) {
      //we timed out waiting for an updated position; stop moving
      lostPositionTimestamp = 0;
      motors.setMotors(0, 0);
      moving = false;
//      SerialUSB.println("Stopped moving.");
      return;
    }
  }
  else
    lostPositionTimestamp = 0;

  if (!moving) {
    moving = true;
    currentCommand = 0;
    commands[0]->start();
    lastCorrectionTime = currentTime;
    return;
  }

  if (currentTime - lastCorrectionTime < AUTODRIVE_CORRECTION_INTERVAL_MS) {
//    skipCount++;
    return;
  }
//  SerialUSB.println(skipCount);
//  skipCount = 0;
  lastCorrectionTime += AUTODRIVE_CORRECTION_INTERVAL_MS;

  lighthouse.recalculate();

#ifdef AUTODRIVE_ENABLED
//  SerialUSB.println(currentCommand);
  if (commands[currentCommand]->loop()) {
    //current command completed; start the next command
    currentCommand = (currentCommand+1) % ZIPPY_COMMAND_COUNT;
    commands[currentCommand]->start();
  }
#endif
}

void AutoDriveMode::stopMoving()
{
}

AutoDriveMode::~AutoDriveMode()
{
  for (int i = 0; i < ZIPPY_COMMAND_COUNT; i++)
    delete commands[i];
  delete[] commands;
}


