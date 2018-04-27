
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
  KVector2* pathPoints;
  int pathPointCount;

  //the point where the robot started traveling on this path
  KVector2 firstPosition;

  //the vector representing the start of the current segment
  KVector2* currentSegmentStart;
  
  //the index into the pathPoints of the point representing the end of the current segment
  int currentSegmentEndIndex = 0;
  
  //the vector from the start to the end of the current segment
  KVector2 currentSegment;

  //the distance we've driven along the current segment
  double currentDistanceAlongSegment = 0.0d;

  double leftSetPoint = 0.0d;
  double leftInput = 0.0d;
  double leftOutput = 0.0d;
  PID leftPID;

  double rightSetPoint = 0.0d;
  double rightInput = 0.0d;
  double rightOutput = 0.0d;
  PID rightPID;
  
//  double calculateInput(LighthouseSensor* sensor, KVector2* nextPosition);
  void updateInputs();
  void calculateNextPosition(KVector2* nextPosition);
  void getCurrentTargetPosition(KVector2* nextPosition);
  
public:
  FollowPath(KVector2* pathPoints, int pathPointCount, int sampleTime);
  void start();
  bool loop();

};

