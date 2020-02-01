
#ifndef _TUNINGCONTROLLER_H_
#define _TUNINGCONTROLLER_H_

#include "ZippyController.h"
// #include "RoutineController.h"
#include "zippies/paths/Routine.h"
#include "PathFollowingController.h"
#include "zippies/config/PIDConfig.h"
#include "zippies/math/StatisticsAccumulator.h"

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#endif

typedef enum class _TuningVariable
{
  DeadZones,
  Proportional,
  Integral,
  Derivative,
} TuningVariable;

typedef enum class _TestingState
{
  PreSyncingWithPreamble,
  MovingIntoPlace,
  // WaitingForFullStop,
  // SyncingWithPreamble,
  Testing,
  PostSyncingWithPreamble,
} TestingState;

typedef struct _AutoTuneTest
{
  double Kp = PID_KP;
  double Ki = PID_KI;
  double Kd = PID_KD;
  double error = 1000.0d;
} AutoTuneTest;

class PIDTuningController : public ZippyController
{

private:
  SensorFusor* sensors;
  Routine routine;
  PathFollowingController pathFollowingController;
  KMatrix2 previousTargetPosition;

  StatisticsAccumulator statisticsAccumulator;

  TuningVariable currentTuningVariable = TuningVariable::Proportional;
  double* currentTestValue;
  double* bestTestValue;
  double currentTestIncrement = 0.0d;

  TestingState currentTestingState = TestingState::PreSyncingWithPreamble;
  AutoTuneTest currentTestRun;
  AutoTuneTest bestTestRun;

#ifdef ENABLE_SDCARD_LOGGING
  bool sdCardInserted = false;
  File logFile;
  int logNumber = 0;

  void openLogFile(unsigned long currentTime);
  void writeLogData(unsigned long currentTime);
  void closeLogFile();
#endif

  void planMoveIntoPlace(const KMatrix2* fromPosition);
  void startMoveIntoPlace(unsigned long currentTime);
  void setupTuning(TuningVariable tuningVariable);
  void startErrorCapture();
  void captureError();
  void stopErrorCapture();
  void evaluateTuning();
  void displayTestData(
      uint8_t y,
      const char* pl, double p,
      const char* il, double i,
      const char* dl, double d);

public:
  // PIDTuningController(SensorFusor* s, Zippy* z);
  PIDTuningController(SensorFusor* s);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
