
#ifndef _AUTODRIVEMODE_H_
#define _AUTODRIVEMODE_H_

#include "Zippy.h"
#include "commands/Commands.h"
#include "lighthouse/KVector2.h"

// #define BEZIER_CONTROL_POINT_COUNT 2*(PATH_POINT_COUNT-1)

class AutoDriveMode
{

private:
  Zippy* zippy;
  unsigned long lostPositionTimestamp;
  bool moving;

  KVector2** pathPoints;
  // KVector2 bezierControlPoints[BEZIER_CONTROL_POINT_COUNT];
  ZippyCommand** commands;
  int currentCommand;

  void computeControlPoints();
  void stopMoving();

public:
  AutoDriveMode(Zippy* zippy);
  ~AutoDriveMode();

  uint8_t getIndicatorColor() { return 0/*TS_8b_Blue*/; }
  void loop();

};

#endif
