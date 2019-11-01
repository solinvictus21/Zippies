
#include "LighthouseSensor.h"
#include "KVector3.h"
#include "../ZippyConfig.h"

// #define DEBUG_LIGHTHOUSE_ERRORS 1
// #define DEBUG_LIGHTHOUSE_EDGES  1

//timings for 48 MHz
//this is the portion of the total sweep cycle duration during which we will reject echoes and reflections from the incoming signal
#define SWEEP_DURATION_EPSILON      3000
#define SWEEP_HIT_MAX_DURATION      1500

void LighthouseSensor::restart()
{
  sensorInput->hitTickWritePtr = sensorInput->hitTickBuffer;
  sensorInput->hitTickReadPtr = sensorInput->hitTickEndPtr;
  previousTickCount = 0;

  currentAxis = 0;
  memset(&currentHitCycles, 0, sizeof(currentHitCycles));
  currentCycleTimeStamp = 0;
  memset(&completedHitCycles, 0, sizeof(completedHitCycles));
  completedCycleTimeStamp = 0;
  currentCaptureState = HitCycleState::Unknown;
}

void LighthouseSensor::loop(unsigned long currentTime)
{
#ifdef DEBUG_LIGHTHOUSE_ERRORS
  //ensure we didn't overflow our buffer; show a warning if so
  if (sensorInput->hitTickWritePtr == sensorInput->hitTickReadPtr) {
    SerialUSB.print(debugNumber);
    SerialUSB.println(" WARNING: Buffer overflow. Potential missed frames.");
  }
#endif

// /*
  if (currentCycleTimeStamp &&
      currentTime - currentCycleTimeStamp >= 2.0d * (SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON))
  {
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    if (debugNumber == 0)
      SerialUSB.println("Lost Lighthouse signal.");
#endif
    resetSync();
  }
// */

  //this is volatile, so grab it first
  // int processedHitCount = 0;
  unsigned int* hitTickWritePtr = sensorInput->hitTickWritePtr;
  do {
    //we read behind the writer, and updates to our read pointer must be atomic, so check that we're not at the
    //end of the buffer before updating the read pointer to the next value to be read
    unsigned int* nextReadPtr = sensorInput->hitTickReadPtr;
    if (nextReadPtr == sensorInput->hitTickEndPtr)
      nextReadPtr = sensorInput->hitTickBuffer;
    else
      nextReadPtr++;

    //exit when the buffer is empty
    if (nextReadPtr == hitTickWritePtr)
      break;

    // processedHitCount++;
    //then let the read pointer move forward; must be atomic, hence the temp pointer above until we confirmed we can move forward
    sensorInput->hitTickReadPtr = nextReadPtr;

    //get our next tick count delta
    unsigned int currentTickCount = *nextReadPtr;
    unsigned int deltaTickCount = calculateDeltaTicks(previousTickCount, currentTickCount);
    previousTickCount = currentTickCount;
    // SerialUSB.println(deltaTickCount);

    switch (currentCaptureState) {
      case HitCycleState::Unknown:
        if (reacquireSync(deltaTickCount))
          currentCycleTimeStamp = currentTime;
        break;

      case HitCycleState::SyncPulseReceived:
        if (checkForHit(deltaTickCount)) {
          memcpy(&completedHitCycles, &currentHitCycles, sizeof(currentHitCycles));
          completedCycleTimeStamp = currentCycleTimeStamp;
        }
        break;

      case HitCycleState::HitPulseStarted:
        if (processHitPulseEnd(deltaTickCount)) {
          memcpy(&completedHitCycles, &currentHitCycles, sizeof(currentHitCycles));
          completedCycleTimeStamp = currentCycleTimeStamp;
        }
        break;

      case HitCycleState::HitPulseReceived:
        processSweepEnd(deltaTickCount);
        break;

      case HitCycleState::SweepCycleEnded:
        if (processSyncPulse(deltaTickCount))
          currentCycleTimeStamp = currentTime;
        break;
    }
  } while (true);

  // if (processedHitCount > 2) {
    // SerialUSB.print("Processed hits: ");
    // SerialUSB.println(processedHitCount);
  // }
}

void LighthouseSensor::resetSync()
{
  currentCycleTimeStamp = 0;
  completedCycleTimeStamp = 0;
  currentCaptureState = HitCycleState::Unknown;
}

bool LighthouseSensor::reacquireSync(unsigned int deltaTickCount)
{
  if (deltaTickCount < SYNC_PULSE_MIN || deltaTickCount >= SYNC_PULSE_MAX)
    return false;

  //begin sync on X axis (0)
  if (((deltaTickCount - SYNC_PULSE_MIN) / SYNC_PULSE_AXIS_WINDOW) & 0x1)
    return false;

  //found a sync signal; capture the sync data and go back to lock-step tracking
  currentAxis = 0;
  currentHitCycles[currentAxis].syncTicks = deltaTickCount;
  currentCaptureState = HitCycleState::SyncPulseReceived;
  return true;
}

bool LighthouseSensor::processSyncPulse(unsigned int deltaTickCount)
{
  if (deltaTickCount < SYNC_PULSE_MIN || deltaTickCount >= SYNC_PULSE_MAX) {
    //we missed an expected sync signal
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Missed sync falling edge: ");
      SerialUSB.println(deltaTickCount);
    }
#endif

    //start over
    resetSync();
    return false;
  }

  //sanity check to ensure we're receiving the sync signal for the axis that we're expecting
  int detectedAxis = ((deltaTickCount - SYNC_PULSE_MIN) / 500) & 0x1;
  if (detectedAxis != currentAxis) {
    //wrong sync pulse; this is for the other axis
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Got wrong sync: ");
      SerialUSB.println(deltaTickCount);
    }
#endif

    //start over
    resetSync();
    return false;
  }

  currentHitCycles[currentAxis].syncTicks = deltaTickCount;
  currentCaptureState = HitCycleState::SyncPulseReceived;
  return !currentAxis;
}

bool LighthouseSensor::checkForHit(unsigned int deltaTickCount)
{
  //this should be the rising edge of the sweep hit
  // unsigned long totalSweepLength = cycleData[currentAxis].pendingSyncTicks + deltaTickCount;
  unsigned long totalSweepLength = currentHitCycles[currentAxis].syncTicks + deltaTickCount;
  // /*
  if (totalSweepLength >= SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON) {
    //we missed the sweep hit for this axis, and the total sweep length is long enough that we now
    //have no idea which signal we'll receive next
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Lost signal lock: ");
      SerialUSB.println(totalSweepLength);
    }
#endif
    resetSync();
    return false;
  }
  // */

  if (totalSweepLength >= SWEEP_DURATION_TICK_COUNT - SWEEP_DURATION_EPSILON) {
    //we missed the sweep hit
#ifdef DEBUG_LIGHTHOUSE_MISSES
    if (debugNumber == 0) {
      SerialUSB.print("Missed sweep hit: ");
      SerialUSB.print(currentHitCycles[currentAxis].syncTicks);
      SerialUSB.print(" ");
      SerialUSB.print(deltaTickCount);
      SerialUSB.print(" ");
      SerialUSB.println(totalSweepLength);
    }
#endif

    currentHitCycles[currentAxis].sweepHitStartTicks = 0;
    currentHitCycles[currentAxis].sweepHitEndTicks = 0;
    currentAxis = (currentAxis + 1) & 0x1;
    currentCaptureState = HitCycleState::SweepCycleEnded;
    return currentAxis;
  }

  //this sweep hit is hitting within the expected sweep window; wait for the falling edge
  currentHitCycles[currentAxis].sweepHitStartTicks = deltaTickCount;
  currentCaptureState = HitCycleState::HitPulseStarted;
  return false;
}

bool LighthouseSensor::processHitPulseEnd(unsigned int deltaTickCount)
{
  // /*
  if (deltaTickCount >= SWEEP_HIT_MAX_DURATION) {
    //we missed the end of the hit pulse
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Missed sweep hit falling edge: ");
      SerialUSB.println(deltaTickCount);
    }
#endif
    // cycleData[currentAxis].pendingSweepHitTicks = 0;
    currentHitCycles[currentAxis].sweepHitEndTicks = 0;
    processSweepEnd(deltaTickCount);
    return currentAxis;
  }
  // */

  currentHitCycles[currentAxis].sweepHitEndTicks = deltaTickCount;
  currentCaptureState = HitCycleState::HitPulseReceived;
  return currentAxis;
}

void LighthouseSensor::processSweepEnd(unsigned int deltaTickCount)
{
// SerialUSB.print("Sweep end: ");
// SerialUSB.println(deltaTickCount);
  unsigned long totalCycleLength = currentHitCycles[currentAxis].syncTicks +
      currentHitCycles[currentAxis].sweepHitStartTicks +
      currentHitCycles[currentAxis].sweepHitEndTicks +
      deltaTickCount;
  if (totalCycleLength < SWEEP_DURATION_TICK_COUNT - SWEEP_DURATION_EPSILON) {
    //the next rising pulse edge was detected before it should have been due to what appear to be shortcomings in the
    //current build of the lighthouse circuit; reject these pulses as "echoes"
    //in this particular case, the 2.2pF capacitors during transimpedance amplification appear to be generating the pulse
    //echoes; the next circuit build will use 5pF capacitors, which eliminates the pulse echoes between the falling edge
    //of the last sweep hit and the rising edge of the next sync pulse without reducing overall circuit sensitivity
#ifdef DEBUG_LIGHTHOUSE_EDGES
    SerialUSB.println("Ignoring signal echo");
//    cycleData[currentAxis].edgeEchoCount++;
#endif
    return;
  }
  else if (totalCycleLength >= SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON) {
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Total sweep length too long: ");
      SerialUSB.print(currentHitCycle.syncTicks);
      SerialUSB.print(" ");
      SerialUSB.print(currentHitCycle.sweepHitStartTicks);
      SerialUSB.print(" ");
      SerialUSB.print(currentHitCycle.sweepHitEndTicks);
      SerialUSB.print(" ");
      SerialUSB.print(deltaTickCount);
      SerialUSB.print(" ");
      SerialUSB.println(totalCycleLength);
    }
#endif

    resetSync();
    return;
  }

  //found rising edge of the sync pulse
  // currentAxis = (currentAxis+1) & 0x1;
  currentAxis = (currentAxis + 1) & 0x1;
  currentCaptureState = HitCycleState::SweepCycleEnded;
}

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks)
{
  //calculate the delta between the ticks; they are derived from a 24-bit counter, so we must be cautious to check when it rolls over
  return startTicks > endTicks
    ? (0x01000000 - startTicks) + endTicks
    : endTicks - startTicks;
}
