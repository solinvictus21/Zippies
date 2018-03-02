
#pragma once

#include <PID_v1.h>
#include "KVector.h"

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

class MoveTowardPoint : public ZippyCommand
{

private:
  KVector2 currentTargetPosition;

  double leftSetPoint = 0.0d;
  double leftInput = 0.0d;
  double leftOutput = 0.0d;
  PID leftPID;

  double rightSetPoint = 0.0d;
  double rightInput = 0.0d;
  double rightOutput = 0.0d;
  PID rightPID;
  
  void updateInputs();
  
public:
  MoveTowardPoint(double x, double y);
  void start();
  bool loop();

};

class FollowPath : public ZippyCommand
{

private:
  KVector2 currentTargetPosition;

  double leftSetPoint = 0.0d;
  double leftInput = 0.0d;
  double leftOutput = 0.0d;
  PID leftPID;

  double rightSetPoint = 0.0d;
  double rightInput = 0.0d;
  double rightOutput = 0.0d;
  PID rightPID;
  
  void updateInputs();
  
public:
  FollowPath(double x, double y);
  void start();
  bool loop();

};


