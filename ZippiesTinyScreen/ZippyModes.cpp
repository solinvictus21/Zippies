
#include "ZippyModes.h"
#include "Bluetooth.h"
#include "ZippyFace.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

#define AUTODRIVE_ENABLED

#define AUTODRIVE_MISSING_POSITION_TIMEOUT    1000
#define AUTODRIVE_CORRECTION_INTERVAL_MS       200
#define AUTODRIVE_REAR_POSITION               -800.0d
#define AUTODRIVE_FRONT_POSITION                 0.0d
#define AUTODRIVE_LEFT_POSITION               -600.0d
#define AUTODRIVE_RIGHT_POSITION               600.0d

#define ZIPPY_COMMAND_COUNT 20

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
  commands = new ZippyCommand*[ZIPPY_COMMAND_COUNT];
  commands[0] = new Pause(3.0d);
  commands[1] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_REAR_POSITION);
  commands[2] = new Pause(3.0d);
  commands[3] = new MoveTowardPoint(AUTODRIVE_LEFT_POSITION, AUTODRIVE_REAR_POSITION);
  commands[4] = new Pause(3.0d);
  commands[5] = new MoveTowardPoint(AUTODRIVE_LEFT_POSITION, AUTODRIVE_FRONT_POSITION);
  commands[6] = new Pause(3.0d);
  commands[7] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_FRONT_POSITION);
  commands[8] = new Pause(3.0d);
  commands[9] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_REAR_POSITION);
  commands[10] = new Pause(3.0d);

  commands[11] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_FRONT_POSITION);
  commands[12] = new Pause(3.0d);
  commands[13] = new MoveTowardPoint(AUTODRIVE_LEFT_POSITION, AUTODRIVE_FRONT_POSITION);
  commands[14] = new Pause(3.0d);
  commands[15] = new MoveTowardPoint(AUTODRIVE_LEFT_POSITION, AUTODRIVE_REAR_POSITION);
  commands[16] = new Pause(3.0d);
  commands[17] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_REAR_POSITION);
  commands[18] = new Pause(3.0d);
  commands[19] = new MoveTowardPoint(AUTODRIVE_RIGHT_POSITION, AUTODRIVE_FRONT_POSITION);
}

AutoDriveMode::~AutoDriveMode()
{
  for (int i = 0; i < ZIPPY_COMMAND_COUNT; i++)
    delete commands[i];
  delete[] commands;
}

void AutoDriveMode::loop()
{
  if (currentCommand >= ZIPPY_COMMAND_COUNT)
    return;
  
  unsigned long currentTime = millis();
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

  if (currentTime - lastCorrectionTime < AUTODRIVE_CORRECTION_INTERVAL_MS)
    return;
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



