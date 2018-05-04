
#pragma once
#include <Tinyscreen.h>
#include "ZippyCommand.h"
#include "KVector.h"

#define PATH_POINT_COUNT 19
#define BEZIER_CONTROL_POINT_COUNT 2*(PATH_POINT_COUNT-1)

class ZippyMode
{

public:
  virtual uint8_t getIndicatorColor() = 0;
  virtual void loop() = 0;
  
};

class AutoDriveMode : public ZippyMode
{

private:
  unsigned long lostPositionTimestamp;
  bool moving;

  unsigned long lastCorrectionTime;

  KVector2 pathPoints[PATH_POINT_COUNT];
  KVector2 bezierControlPoints[BEZIER_CONTROL_POINT_COUNT];
  ZippyCommand** commands;
  int currentCommand;

  void computeControlPoints();
  void stopMoving();

public:
  AutoDriveMode();
  ~AutoDriveMode();

  uint8_t getIndicatorColor() { return TS_8b_Blue; }
  void loop();
  
};

class UserDriveMode : public ZippyMode
{

public:
  UserDriveMode();
  
  uint8_t getIndicatorColor() { return TS_8b_Green; }
  void loop();
  
};

