
#ifndef _ZTUNINGCONTROLLER_H_
#define _ZTUNINGCONTROLLER_H_

#include "ZController.h"
#include "ZRoutineController.h"

#ifdef ENABLE_SDCARD_LOGGING
#include <SD.h>
#endif

typedef enum class _TuningVariable
{
  Proportional,
  Integral,
  Derivative,
} TuningVariable;

typedef enum class _TestingState
{
  MovingIntoPlace,
  WaitingForFullStop,
  SyncingWithPreamble,
  Testing,
} TestingState;

typedef struct _AutoTuneTest
{
  double Kp = 0.0d;
  double Ki = 0.0d;
  double Kd = 0.0d;
  double error = 0.0d;
  double range = 0.0d;
} AutoTuneTest;

class ZTuningController : public ZController
{

private:
  SensorFusor* sensors;
  Zippy* zippy;

  ZRoutineController routineController;

  TuningVariable currentTuningVariable = TuningVariable::Proportional;
  double currentTestIncrement = 0.0d;

  TestingState currentTestingState = TestingState::MovingIntoPlace;
  double* currentTestValue = NULL;
  AutoTuneTest currentTestRun;
  double* currentBestTestValue = NULL;
  AutoTuneTest bestTestRun;

  double errorAccumulator = 0.0d;
  unsigned long errorCounter = 0;

#ifdef ENABLE_SDCARD_LOGGING
  bool sdCardInserted = false;
  File logFile;
  int logNumber = 0;

  void openLogFile(unsigned long currentTime);
  void writeLogData(unsigned long currentTime);
  void closeLogFile();
#endif

  void setupTuning(TuningVariable tuningVariable);
  void evaluateTuning();
  void displayTestData(uint8_t x, double p, double i, double d, double e, double r);

public:
  ZTuningController(SensorFusor* s, Zippy* z);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
