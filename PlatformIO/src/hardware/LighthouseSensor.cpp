
#include "zippies/hardware/LighthouseSensor.h"
#include "zippies/ZippyMath.h"

// #define DEBUG_LIGHTHOUSE_SYNC     1
// #define DEBUG_LIGHTHOUSE_ERRORS   1
// #define DEBUG_LIGHTHOUSE_MISSES   1

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
  memset(&completedHitCycles, 0, sizeof(completedHitCycles));
  completedCycleTimeStamp = 0;
  currentCaptureState = HitCycleState::Unknown;
}

void LighthouseSensor::loop(unsigned long currentTime)
{
  // SerialUSB.println("Looping sensor.");
#ifdef DEBUG_LIGHTHOUSE_ERRORS
  //ensure we didn't overflow our buffer; show a warning if so
  if (sensorInput->hitTickWritePtr == sensorInput->hitTickReadPtr) {
    SerialUSB.print(debugNumber);
    SerialUSB.println(" WARNING: Buffer overflow. Potential missed frames.");
  }
#endif

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

    //get our next tick count prior to updating the read pointer so that our interrupt doesn't step on our value while
    //we're still in the process of retrieving it
    unsigned int currentTickCount = *nextReadPtr;

    //then let the read pointer move forward; must be atomic, hence the temp pointer above until we confirmed we can move forward
    sensorInput->hitTickReadPtr = nextReadPtr;

    unsigned int deltaTickCount = calculateDeltaTicks(previousTickCount, currentTickCount);
    // SerialUSB.print("Processing pulse: ");
    // SerialUSB.println(deltaTickCount);
    /*
    if (deltaTickCount >= SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON) {
#ifdef DEBUG_LIGHTHOUSE_ERRORS
      SerialUSB.print(debugNumber);
      SerialUSB.print(" Got invalid tick count: ");
      SerialUSB.print(nextReadPtr - hitTickWritePtr);
      SerialUSB.print(" ");
      SerialUSB.print(previousTickCount);
      SerialUSB.print(" ");
      SerialUSB.print(currentTickCount);
      SerialUSB.print(" ");
      SerialUSB.println(deltaTickCount);
#endif
      resetSync();
    }
    // */
    previousTickCount = currentTickCount;

    switch (currentCaptureState) {
      case HitCycleState::Unknown:
        // SerialUSB.println("Sensor in unknown state.");
        reacquireSync(deltaTickCount);
        break;

      case HitCycleState::SyncPulseReceived:
        // SerialUSB.println("Sensor sync pulse received.");
        if (processHitPulseStart(deltaTickCount)) {
          memcpy(&completedHitCycles, &currentHitCycles, sizeof(currentHitCycles));
          completedCycleTimeStamp = currentTime;
        }
        break;

      case HitCycleState::HitPulseStarted:
        if (processHitPulseEnd(deltaTickCount)) {
          memcpy(&completedHitCycles, &currentHitCycles, sizeof(currentHitCycles));
          completedCycleTimeStamp = currentTime;
        }
        break;

      case HitCycleState::HitPulseReceived:
        if (processSweepEnd(deltaTickCount)) {
          memcpy(&completedHitCycles, &currentHitCycles, sizeof(currentHitCycles));
          completedCycleTimeStamp = currentTime;
        }
        break;

      case HitCycleState::SweepCycleEnded:
        processSyncPulseEnd(deltaTickCount);
        break;
    }
  } while (true);

  // SerialUSB.println("Ended sensor loop.");
  /*
  if (processedHitCount > 2) {
    SerialUSB.print("Processed hits: ");
    SerialUSB.println(processedHitCount);
  }
  // */

/*
  if (currentCycleTimeStamp &&
      currentTime - currentCycleTimeStamp >= 20)
  {
    resetSync();
  }
// */
}

void LighthouseSensor::resetSync()
{
#ifdef DEBUG_LIGHTHOUSE_SYNC
  SerialUSB.print(debugNumber);
  SerialUSB.println(" Lost sync lock.");
#endif
  completedCycleTimeStamp = 0;
  currentCaptureState = HitCycleState::Unknown;
}

void LighthouseSensor::reacquireSync(unsigned int deltaTickCount)
{
  // SerialUSB.print(debugNumber);
  // SerialUSB.print(" Attempting to reacquire on pulse length: ");
  // SerialUSB.println(deltaTickCount);
  if (deltaTickCount < SYNC_PULSE_MIN || deltaTickCount >= SYNC_PULSE_MAX)
    return;

  //begin sync on X axis (0)
  if (SYNC_PULSE_AXIS(deltaTickCount))
    return;

#ifdef DEBUG_LIGHTHOUSE_SYNC
    SerialUSB.print(debugNumber);
    SerialUSB.println(" Acquired sync lock.");
#endif

  //found a sync signal; capture the sync data and go back to lock-step tracking
  currentAxis = 0;
  // SerialUSB.print("sync pulse length: ");
  // SerialUSB.print(deltaTickCount);
  // SerialUSB.print(" ");
  currentHitCycles[currentAxis].syncTicks = deltaTickCount;
  currentCaptureState = HitCycleState::SyncPulseReceived;
}

void LighthouseSensor::processSyncPulseEnd(unsigned int deltaTickCount)
{
  if (deltaTickCount < SYNC_PULSE_MIN || deltaTickCount >= SYNC_PULSE_MAX) {
    //we missed an expected sync signal
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Missed sync falling edge: ");
    SerialUSB.println(deltaTickCount);
#endif

    //start over
    resetSync();
    return;
  }

  //sanity check to ensure we're receiving the sync signal for the axis that we're expecting
  // int detectedAxis = ((deltaTickCount - SYNC_PULSE_MIN) / SYNC_PULSE_AXIS_WINDOW) & 0x1;
  int detectedAxis = SYNC_PULSE_AXIS(deltaTickCount);
  if (detectedAxis != currentAxis) {
    //wrong sync pulse; this is for the other axis
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Got unexpected sync signal axis: ");
    SerialUSB.println(deltaTickCount);
#endif

    //start over
    resetSync();
    return;
  }

  // SerialUSB.print("sync pulse length: ");
  // SerialUSB.print(deltaTickCount);
  // SerialUSB.print(" ");
  currentHitCycles[currentAxis].syncTicks = deltaTickCount;
  currentCaptureState = HitCycleState::SyncPulseReceived;
}

bool LighthouseSensor::processHitPulseStart(unsigned int deltaTickCount)
{
  //this should be the rising edge of the sweep hit
  // unsigned long totalSweepLength = cycleData[currentAxis].pendingSyncTicks + deltaTickCount;
  unsigned long totalCycleLength = currentHitCycles[currentAxis].syncTicks + deltaTickCount;
  if (totalCycleLength >= SWEEP_DURATION_TICK_COUNT - SWEEP_DURATION_EPSILON) {
    //we missed the sweep hit
#ifdef DEBUG_LIGHTHOUSE_MISSES
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Missed sweep hit: ");
    SerialUSB.print(currentHitCycles[currentAxis].syncTicks);
    SerialUSB.print(" ");
    SerialUSB.println(deltaTickCount);
#endif

    currentHitCycles[currentAxis].sweepHitStartTicks = 0;
    currentHitCycles[currentAxis].sweepHitEndTicks = 0;
    return processSweepEnd(totalCycleLength);
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
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Missed sweep hit falling edge: ");
    SerialUSB.println(deltaTickCount);
#endif

    currentHitCycles[currentAxis].sweepHitEndTicks = 0;
    return processSweepEnd(
        currentHitCycles[currentAxis].syncTicks +
        currentHitCycles[currentAxis].sweepHitStartTicks +
        deltaTickCount);
  }
  // */

  currentHitCycles[currentAxis].sweepHitEndTicks = deltaTickCount;
  currentCaptureState = HitCycleState::HitPulseReceived;
  return false;
}

bool LighthouseSensor::processSweepEnd(unsigned int totalSweepTickCount)
{
  // SerialUSB.print("Sweep end: ");
  // SerialUSB.println(deltaTickCount);
  /*
  if (totalSweepTickCount < SWEEP_DURATION_TICK_COUNT - SWEEP_DURATION_EPSILON) {
    //the next rising pulse edge was detected before it should have been due to what appear to be shortcomings in the
    //current build of the lighthouse circuit; reject these pulses as "echoes"
    //in this particular case, the 2.2pF capacitors during transimpedance amplification appear to be generating the pulse
    //echoes; the next circuit build will use 5pF capacitors, which eliminates the pulse echoes between the falling edge
    //of the last sweep hit and the rising edge of the next sync pulse without reducing overall circuit sensitivity
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Ignoring signal echo: ");
    SerialUSB.print(currentHitCycles[currentAxis].syncTicks);
    SerialUSB.print(" ");
    SerialUSB.print(currentHitCycles[currentAxis].sweepHitStartTicks);
    SerialUSB.print(" ");
    SerialUSB.print(currentHitCycles[currentAxis].sweepHitEndTicks);
    SerialUSB.print(" ");
    SerialUSB.print(deltaTickCount);
    SerialUSB.print(" ");
    SerialUSB.println(totalCycleLength);
#endif
    return false;
  }
  else
  */
  if (totalSweepTickCount >= SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON) {
#ifdef DEBUG_LIGHTHOUSE_ERRORS
    SerialUSB.print(debugNumber);
    SerialUSB.print(" Total sweep length too long: ");
    SerialUSB.print(currentHitCycles[currentAxis].syncTicks);
    SerialUSB.print(" ");
    SerialUSB.print(currentHitCycles[currentAxis].sweepHitStartTicks);
    SerialUSB.print(" ");
    SerialUSB.print(currentHitCycles[currentAxis].sweepHitEndTicks);
    SerialUSB.print(" ");
    SerialUSB.print(deltaTickCount);
    SerialUSB.print(" ");
    SerialUSB.println(totalCycleLength);
#endif

    resetSync();
    return false;
  }

  //found rising edge of the sync pulse
  // currentAxis = (currentAxis+1) & 0x1;
  currentAxis = (currentAxis + 1) & 0x1;
  currentCaptureState = HitCycleState::SweepCycleEnded;
  return !currentAxis;
}

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks)
{
  //calculate the delta between the ticks; they are derived from a 24-bit counter, so we must be cautious to check when it rolls over
  return startTicks > endTicks
    ? (0x01000000 - startTicks) + endTicks
    : endTicks - startTicks;
}
