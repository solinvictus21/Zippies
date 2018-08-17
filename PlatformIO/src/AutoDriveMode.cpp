
#include <SPI.h>
#include "AutoDriveMode.h"
#include "AutoDriveData.h"
#include "commands/Commands.h"
#include "ZippyConfig.h"

#define AUTODRIVE_ENABLED 1
#if ZIPPY_ID != 0
#define PUT_YA_THING_DOWN_FLIP_IT_AND_REVERSE_IT 1
#endif
#define ZIPPY_SPACING_MM  60.0d

#define AUTODRIVE_MISSING_POSITION_TIMEOUT     200
#define AUTODRIVE_LIMITS_OFFSETY              1200.0d

#define PATH_POINT_SCALE_FACTOR               1000.0d

#define ZIPPY_COMMAND_COUNT 2

// extern ZippyFace face;
// extern Bluetooth bluetooth;
// extern Lighthouse lighthouse;
// extern MotorDriver motors;

AutoDriveMode::AutoDriveMode(Zippy* z)
  : zippy(z),
    lostPositionTimestamp(0),
    moving(false),
    currentCommand(0)
{
  //figure 8 from center
  // pathPoints = new KVector2*[PATH_POINT_COUNT];
  for (int i = 0; i < PATH_POINT_COUNT; i++) {
    // double x = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i][0];
    // double y = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i][1];
    double x = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i].getX();
    double y = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i].getY();
#ifdef PUT_YA_THING_DOWN_FLIP_IT_AND_REVERSE_IT
    //flip along the x and y axes
    x = -x;
    y = -y;
#endif
    // pathPoints[i] = new KVector2(x, y + AUTODRIVE_LIMITS_OFFSETY);
    PATH_POINTS[i].set(x, y + AUTODRIVE_LIMITS_OFFSETY);
  }

/*
#ifdef PUT_YA_THING_DOWN_FLIP_IT_AND_REVERSE_IT
  for (int i = 0; i < PATH_POINT_COUNT/2; i++) {
    double tmpX = PATH_POINTS[i].getX();
    double tmpY = PATH_POINTS[i].getY();
    PATH_POINTS[i].set(&PATH_POINTS[PATH_POINT_COUNT - i - 1]);
    PATH_POINTS[PATH_POINT_COUNT - i - 1].set(tmpX, tmpY);
  }
#endif
*/

  commands = new ZippyCommand*[ZIPPY_COMMAND_COUNT];
  // commands[0] = new Pause(zippy, 2.0d);
  commands[0] = new SyncWithPreamble(zippy);
  // commands[1] = new FollowPath(zippy, pathPoints, PATH_POINT_COUNT);
  commands[1] = new FollowPath(zippy, PATH_POINTS, PATH_POINT_COUNT);
}

void AutoDriveMode::loop()
{
  if (currentCommand >= ZIPPY_COMMAND_COUNT)
    return;

//  static int skipCount = 0;
  unsigned long currentTime = millis();
  if (!zippy->hasLighthouseSignal()) {
    if (!moving)
      return;

    //the sensors do not have the latest robot position; wait until they do; if we're currently in motion, setup a
    //timeout to stop moving if we go too long without the position information
    if (!lostPositionTimestamp)
      lostPositionTimestamp = currentTime;
    else if (currentTime - lostPositionTimestamp >= AUTODRIVE_MISSING_POSITION_TIMEOUT) {
      //we timed out waiting for an updated position; stop moving
      lostPositionTimestamp = 0;
      zippy->stop();
      moving = false;
      // SerialUSB.println("Stopped moving.");
      return;
    }
  }
  else
    lostPositionTimestamp = 0;

  if (!moving) {
    moving = true;
    currentCommand = 0;
    commands[0]->start(currentTime);
    // SerialUSB.println("Started moving.");
    return;
  }

#ifdef AUTODRIVE_ENABLED
//  SerialUSB.println(currentCommand);
  if (commands[currentCommand]->loop(currentTime)) {
    //current command completed; start the next command
    currentCommand = (currentCommand+1) % ZIPPY_COMMAND_COUNT;
    commands[currentCommand]->start(currentTime);
  }
#endif
}

void AutoDriveMode::stopMoving()
{
}

AutoDriveMode::~AutoDriveMode()
{
  // for (int i = 0; i < PATH_POINT_COUNT; i++)
    // delete pathPoints[i];
  // delete[] pathPoints;
  for (int i = 0; i < ZIPPY_COMMAND_COUNT; i++)
    delete commands[i];
  delete[] commands;
}
