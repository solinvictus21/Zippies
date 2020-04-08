
#ifndef _MOTORTUNINGCONTROLLER_H_
#define _MOTORTUNINGCONTROLLER_H_

#include <PID_v1.h>

#include "ZippyController.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/MotorDriver.h"
#include "zippies/hardware/ZippyFace.h"
#include "zippies/math/StatisticsAccumulator.h"

#define DEFAULT_TUNING_FACTOR    0.2d

typedef enum class _MotorTuningState
{
  RightWheelForward,
  RightWheelReverse,
  LeftWheelForward,
  LeftWheelReverse,
  Forward,
  Backward,
  TurningLeft,
  TurningRight,
} MotorTuningState;

class MotorTuningController : public ZippyController
{

private:
  SensorFusor* sensors;
  MotorDriver motors;
  ZippyFace face;

  double leftMotorStiction = 0.0d;
  double leftMotorDeadZone = 0.0d;
  StatisticsAccumulator leftMotorStatistics;
  double rightMotorStiction = 0.0d;
  double rightMotorDeadZone = 0.0d;
  StatisticsAccumulator rightMotorStatistics;

  double motorStiction = 0.0d;
  double motorDeadZone = 0.0d;
  // unsigned long successZoneTimeStamp = 0;
  int successIterationCounter = 0;
  double successZoneTotal = 0.0d;
  double successZoneAverage = 0.0d;
  MotorTuningState currentTuningState = MotorTuningState::RightWheelForward;

  /*
  double leftTuningValue = MOTOR_DEAD_ZONE;
  double leftTuningFactor = DEFAULT_TUNING_FACTOR;
  double rightTuningValue = MOTOR_DEAD_ZONE;
  double rightTuningFactor = DEFAULT_TUNING_FACTOR;
  */

  PID velocityPID;
  double velocityInput;
  double velocitySetPoint;
  double velocityOutput;

  KMatrix2 previousPosition;
  KMatrix2 currentVelocity;
  unsigned long previousUpdateTimeStamp;

  StatisticsAccumulator inputAccumulator;
  StatisticsAccumulator outputAccumulator;

  void evaluateForward();
  void displayTestData();
  void displayDeadZoneData();
  bool testComplete(unsigned long currentTime, double testResult, double testMin, double testMax);

public:
  MotorTuningController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
