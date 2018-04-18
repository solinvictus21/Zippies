
#pragma once

#include <PID_v1.h>
#include "KVector.h"
#include "Lighthouse.h"

class ZippyCommand
{

public:
  virtual void start() = 0;
  virtual bool loop() = 0;

};

class Pause : public ZippyCommand
{

private:
  unsigned long deltaTimeMS;
  unsigned long startTimeMS;
  
public:
  Pause(double deltaTimeSeconds);
  void start();
  bool loop();
  
};

class FollowPath : public ZippyCommand
{

private:
  KVector2 firstPosition;
  KVector2* previousPosition;
  KVector2 deltaPosition;
  KVector2* pathPoints;
  int pathPointCount;
  int currentPathPoint;
  double distanceDrivenAlongPath = 0.0d;

  double leftSetPoint = 0.0d;
  double leftInput = 0.0d;
  double leftOutput = 0.0d;
  PID leftPID;

  double rightSetPoint = 0.0d;
  double rightInput = 0.0d;
  double rightOutput = 0.0d;
  PID rightPID;
  
  double calculateInput(LighthouseSensor* sensor, KVector2* nextPosition);
  void updateInputs();
  
public:
  FollowPath(KVector2* pathPoints, int pathPointCount);
  void start();
  bool loop();

};

