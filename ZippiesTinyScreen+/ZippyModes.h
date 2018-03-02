
#pragma once
#include <Tinyscreen.h>
#include "ZippyCommand.h"

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

  ZippyCommand** commands;
  int currentCommand;
  
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

