
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
  double Kp = DEFAULT_KP;
  double Ki = DEFAULT_KI;
  double Kd = DEFAULT_KD;
  double error = 1000.0d;
} AutoTuneTest;

class ZTuningController : public ZController
{

private:
  SensorFusor* sensors;
  Zippy* zippy;

  ZRoutineController routineController;

  TuningVariable currentTuningVariable = TuningVariable::Proportional;
  double* currentTestValue;
  double* bestTestValue;
  double currentTestIncrement = 0.0d;

  TestingState currentTestingState = TestingState::MovingIntoPlace;
  AutoTuneTest currentTestRun;
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
  void displayTestData(
      uint8_t y,
      const char* pl, double p,
      const char* il, double i,
      const char* dl, double d);

public:
  ZTuningController(SensorFusor* s, Zippy* z);

  void start(unsigned long startTime);
  void loop(unsigned long currentTime);
  void stop();

};

#endif
