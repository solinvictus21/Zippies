
#ifndef _LIGHTHOUSESENSOR_H_
#define _LIGHTHOUSESENSOR_H_

#include <Arduino.h>
#include "KVector2.h"
#include "KQuaternion3.h"
#include "Lighthouse.h"
#include "LighthouseOOTX.h"
#include "KVector3.h"
#include "../ZippyConfig.h"

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
  //number of ticks from the end of the sync signal to the start of the sweep hit
  unsigned long syncTicks = 0;
  unsigned long sweepHitStartTicks = 0;
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
  unsigned long currentCycleTimeStamp = 0;
  LighthouseSensorHitCycle completedHitCycles[2];
  unsigned long completedCycleTimeStamp = 0;

  void resetSync();
  bool reacquireSync(unsigned int deltaTickCount);
  bool processSyncPulse(unsigned int deltaTickCount);
  bool checkForHit(unsigned int deltaTickCount);
  bool processHitPulseEnd(unsigned int deltaTickCount);
  void processSweepEnd(unsigned int deltaTickCount);

  void restart();

  friend class SensorFusor;

  LighthouseSensor() {}
  // LighthouseSensor(LighthouseSensorInput* sensorInput, int debugNumber);

public:
  unsigned int getNextPulse();

  void loop(unsigned long currentTime);

  const LighthouseSensorHitCycle* getXHitCycle() const { return &completedHitCycles[0]; }
  const LighthouseSensorHitCycle* getYHitCycle() const { return &completedHitCycles[1]; }

};

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks);

#endif
