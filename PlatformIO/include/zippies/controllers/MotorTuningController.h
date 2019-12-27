
#ifndef _MOTORTUNINGCONTROLLER_H_
#define _MOTORTUNINGCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/ZippyHardware.h"
#include "zippies/config/MotorConfig.h"
#include "zippies/math/StatisticsAccumulator.h"

#define DEFAULT_TUNING_FACTOR    0.2d

typedef enum class _MotorTuningState
{
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

  MotorTuningState currentTuningState = MotorTuningState::Forward;
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

public:
  MotorTuningController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
