
#ifndef _LIGHTHOUSESENSOR_H_
#define _LIGHTHOUSESENSOR_H_

#include <Arduino.h>

#include "zippies/ZippyMath.h"
#include "LighthouseOOTX.h"

#define BUFFER_SIZE 128

typedef struct _LighthouseSensorInput
{
  unsigned int hitTickBuffer[BUFFER_SIZE];
  unsigned int* const hitTickEndPtr = hitTickBuffer + BUFFER_SIZE - 1;
  unsigned int* volatile hitTickWritePtr = hitTickBuffer;
  unsigned int* volatile hitTickReadPtr = hitTickEndPtr;
} LighthouseSensorInput;

typedef enum class _HitCycleState
{
    Unknown,
    SyncPulseReceived,
    HitPulseStarted,
    HitPulseReceived,
    SweepCycleEnded
} HitCycleState;

typedef struct _LighthouseSensorHitCycle
{
  //number of ticks for the sync pulse
  unsigned long syncTicks = 0;
  //number of ticks from the end of the sync signal to the start of the sweep hit
  unsigned long sweepHitStartTicks = 0;
  //number of ticks from the start of the sweep hit to the end of the sweep hit
  unsigned long sweepHitEndTicks = 0;
} LighthouseSensorHitCycle;

class LighthouseSensor
{

private:
  //for debugging
  int debugNumber;

  //raw input shared with the interrupt handler; interrupt handler writes and LighthouseSensor reads and processes
  LighthouseSensorInput* sensorInput;
  unsigned int previousTickCount = 0;

  //the current edge we're expecting within the current cycle
  HitCycleState currentCaptureState = HitCycleState::Unknown;
  // unsigned long currentSyncTime = 0;
  int currentAxis = 0;
  LighthouseSensorHitCycle currentHitCycles[2];
  // unsigned long currentCycleTimeStamp = 0;
  LighthouseSensorHitCycle completedHitCycles[2];
  unsigned long completedCycleTimeStamp = 0;

  void resetSync();
  void reacquireSync(unsigned int deltaTickCount);
  void processSyncPulseEnd(unsigned int deltaTickCount);
  bool processHitPulseStart(unsigned int deltaTickCount);
  bool processHitPulseEnd(unsigned int deltaTickCount);
  bool processSweepEnd(unsigned int deltaTickCount);

  void restart();

  friend class SensorFusor;

  LighthouseSensor() {}

public:
  void loop(unsigned long currentTime);

  const LighthouseSensorHitCycle* getXHitCycle() const { return &completedHitCycles[0]; }
  const LighthouseSensorHitCycle* getYHitCycle() const { return &completedHitCycles[1]; }

};

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks);

#endif
