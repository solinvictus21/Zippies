
#pragma once

#include <Arduino.h>
#include "KVector.h"
#include "KQuaternion.h"

//#define DEBUG_SIGNAL_EDGES 1
#define DEBUG_LIGHTHOUSE_EDGES 1

//#define LIGHTHOUSE_DEBUG_SCREEN_SIGNAL
//#define LIGHTHOUSE_DEBUG_SCREEN_ERRORS
//#define LIGHTHOUSE_DEBUG_SIGNAL
//#define LIGHTHOUSE_DEBUG_ERRORS

#define BUFFER_SIZE 32
#define BASE_STATION_INFO_BLOCK_SIZE 33

typedef struct _RotorFactoryCalibrationData
{
  double phase = 0.0d;
  double curve = 0.0d;
  double tilt = 0.0d;
  double gibbousPhase = 0.0d;
  double gibbousMagnitude = 0.0d;
} RotorFactoryCalibrationData;

typedef struct _LighthouseSensorInput
{
#ifdef DEBUG_SIGNAL_EDGES
  unsigned int volatile risingEdgeCount = 0;
  unsigned int volatile fallingEdgeCount = 0;
#endif
  unsigned int hitTickBuffer[BUFFER_SIZE];
  unsigned int* const hitTickEndPtr = hitTickBuffer + BUFFER_SIZE - 1;
  unsigned int* volatile hitTickWritePtr = hitTickBuffer;
  unsigned int* volatile hitTickReadPtr = hitTickEndPtr;
} LighthouseSensorInput;

enum PulseEdge
{
    Unknown, SyncRising, SyncFalling, SweepRising, SweepFalling
};

typedef struct _SensorCycleData
{
  //pending sync tick count; to be used if we later see the sweep hit
  unsigned long pendingSyncStart = 0;
  unsigned long pendingSyncLength = 0;
  
  //number of ticks in most recent cycle; set to zero when a cycle is missed
  unsigned long syncTickCount = 0;
  //number of ticks from the start of the visible portion of the cycle to the sweep hit
  unsigned long sweepTickCount = 0;
  //updated each time a sweep hit is detected; set to zero when lighthouse signal is unavailable
  unsigned long sweepHitTimeStamp = 0;

#ifdef DEBUG_LIGHTHOUSE_EDGES
  unsigned int syncRisingEdgeHits = 0;
  unsigned int syncRisingEdgeMisses = 0;
  unsigned int syncFallingEdgeHits = 0;
  unsigned int syncFallingEdgeMisses = 0;
  unsigned int sweepRisingEdgeHits = 0;
  unsigned int sweepRisingEdgeMisses = 0;
  unsigned int sweepFallingEdgeHits = 0;  
  unsigned int sweepFallingEdgeMisses = 0;  
#endif

#ifdef LIGHTHOUSE_DEBUG_SCREEN_SIGNAL
  unsigned long lastCycleTickCount = 0;
#elif LIGHTHOUSE_DEBUG_SCREEN_ERRORS
  //error counts
  unsigned long sweepAccumulator = 0;
  unsigned long sweepCounter = 0;
#endif
} SensorCycleData;

class LighthouseSensor
{

private:
  //for debugging
  int debugNumber;

  //raw input shared with the interrupt handler; interrupt handler writes and LighthouseSensor reads and processes
  LighthouseSensorInput* sensorInput;

  //data and code required to process the OOTX frame, for the purpose of extracting the lighthouse orientation
  int zeroCount;
  int syncBitCounter;
  unsigned short payloadLength;
  unsigned short payloadReadMask;
  byte baseStationInfoBlock[BASE_STATION_INFO_BLOCK_SIZE];
  int readInfoBlockIndex;
  byte readInfoBlockMask;
  
  void processOOTXBit(unsigned int syncDelta);
  void calculateLighthousePosition();

  //once the OOTX frame has been found and processed, the lighthouse position and orientation are calculated
  KVector3 lighthousePosition;
  KQuaternion lighthouseOrientation;
  //...and then this flag is set to true
  bool receivedLighthousePosition;

  RotorFactoryCalibrationData xRotor;
  RotorFactoryCalibrationData yRotor;

  //the current edge we're expecting within the current cycle
  PulseEdge currentEdge;
  //the current axis we're processing
  int currentAxis;
  //captured data in each cycle for the x and y axes
  //x = 0; y = 1
  SensorCycleData cycleData[2];
  
  unsigned int previousTickCount;

  //historical data for calculating velocity
  KVector2 previousPositionVector;
  unsigned long previousPositionTimeStamp = 0;

  //position
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  double velocity;
  unsigned long velocityTimeStamp = 0;
  
  void processUnknownPulse(unsigned int startTickCount, unsigned int endTickCount);
  void processSyncPulse(unsigned int startTickCount, unsigned int endTickCount);
  void processSweepHit(unsigned int deltaTickCount);

  bool hasLighthouseSignal() { return receivedLighthousePosition && cycleData[0].sweepHitTimeStamp && cycleData[1].sweepHitTimeStamp; }
  void recalculatePosition();
  void estimatePosition(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long currentTime);
  void recalculateVelocity(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long orientationTimeStamp);

  friend class Lighthouse;

  unsigned int eventCount = 0;
  
public:
  LighthouseSensor(LighthouseSensorInput* sensorInput, int debugNumber);

  void loop();

  //info about the lighthouse position received by this sensor
  int8_t getAccelDirX();
  int8_t getAccelDirY();
  int8_t getAccelDirZ();

  unsigned int getEventCount() {
    unsigned int ec = eventCount;
    eventCount = 0;
    return ec;
  }

#ifdef DEBUG_SIGNAL_EDGES
  unsigned int getRisingEdgeCount() {
    unsigned int v = sensorInput->risingEdgeCount;
    sensorInput->risingEdgeCount = 0;
    return v;
  }

  unsigned int getFallingEdgeCount() {
    unsigned int v = sensorInput->fallingEdgeCount;
    sensorInput->fallingEdgeCount = 0;
    return v;
  }
#endif

#ifdef DEBUG_LIGHTHOUSE_EDGES
  unsigned int getXSyncRisingEdgeHits() {
    unsigned int v = cycleData[0].syncRisingEdgeHits;
    cycleData[0].syncRisingEdgeHits = 0;
    return v;
  }

  unsigned int getXSyncRisingEdgeMisses() {
    unsigned int v = cycleData[0].syncRisingEdgeMisses;
    cycleData[0].syncRisingEdgeMisses = 0;
    return v;
  }

  unsigned int getXSyncFallingEdgeHits() {
    unsigned int v = cycleData[0].syncFallingEdgeHits;
    cycleData[0].syncFallingEdgeHits = 0;
    return v;
  }

  unsigned int getXSyncFallingEdgeMisses() {
    unsigned int v = cycleData[0].syncFallingEdgeMisses;
    cycleData[0].syncFallingEdgeMisses = 0;
    return v;
  }

  unsigned int getXSweepRisingEdgeHits() {
    unsigned int v = cycleData[0].sweepRisingEdgeHits;
    cycleData[0].sweepRisingEdgeHits = 0;
    return v;
  }

  unsigned int getXSweepRisingEdgeMisses() {
    unsigned int v = cycleData[0].sweepRisingEdgeMisses;
    cycleData[0].sweepRisingEdgeMisses = 0;
    return v;
  }

  unsigned int getXSweepFallingEdgeHits() {
    unsigned int v = cycleData[0].sweepFallingEdgeHits;
    cycleData[0].sweepFallingEdgeHits = 0;
    return v;
  }

  unsigned int getXSweepFallingEdgeMisses() {
    unsigned int v = cycleData[0].sweepFallingEdgeMisses;
    cycleData[0].sweepFallingEdgeMisses = 0;
    return v;
  }

  unsigned int getYSyncRisingEdgeHits() {
    unsigned int v = cycleData[1].syncRisingEdgeHits;
    cycleData[1].syncRisingEdgeHits = 0;
    return v;
  }

  unsigned int getYSyncRisingEdgeMisses() {
    unsigned int v = cycleData[1].syncRisingEdgeMisses;
    cycleData[1].syncRisingEdgeMisses = 0;
    return v;
  }

  unsigned int getYSyncFallingEdgeHits() {
    unsigned int v = cycleData[1].syncFallingEdgeHits;
    cycleData[1].syncFallingEdgeHits = 0;
    return v;
  }

  unsigned int getYSyncFallingEdgeMisses() {
    unsigned int v = cycleData[1].syncFallingEdgeMisses;
    cycleData[1].syncFallingEdgeMisses = 0;
    return v;
  }

  unsigned int getYSweepRisingEdgeHits() {
    unsigned int v = cycleData[1].sweepRisingEdgeHits;
    cycleData[1].sweepRisingEdgeHits = 0;
    return v;
  }

  unsigned int getYSweepRisingEdgeMisses() {
    unsigned int v = cycleData[1].sweepRisingEdgeMisses;
    cycleData[1].sweepRisingEdgeMisses = 0;
    return v;
  }

  unsigned int getYSweepFallingEdgeHits() {
    unsigned int v = cycleData[1].sweepFallingEdgeHits;
    cycleData[1].sweepFallingEdgeHits = 0;
    return v;
  }

  unsigned int getYSweepFallingEdgeMisses() {
    unsigned int v = cycleData[1].sweepFallingEdgeMisses;
    cycleData[1].sweepFallingEdgeMisses = 0;
    return v;
  }
#endif

#ifdef LIGHTHOUSE_DEBUG_SCREEN_SIGNAL
  unsigned long getXCycleTickCount() {
    return cycleData[0].lastCycleTickCount;
  }

  unsigned long getYCycleTickCount() {
    return cycleData[1].lastCycleTickCount;
  }

#elif LIGHTHOUSE_DEBUG_SCREEN_ERRORS
  unsigned int getSyncXFallingErrorCount() {
    unsigned int ec = cycleData[0].syncFallingErrorCount;
    cycleData[0].syncFallingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepXRisingErrorCount() {
    unsigned int ec = cycleData[0].sweepRisingErrorCount;
    cycleData[0].sweepRisingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepXFallingErrorCount() {
    unsigned int ec = cycleData[0].sweepFallingErrorCount;
    cycleData[0].sweepFallingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepXAverage() {
    unsigned int avg = (unsigned int)(cycleData[0].sweepAccumulator / cycleData[0].sweepCounter);
    cycleData[0].sweepAccumulator = 0;
    cycleData[0].sweepCounter = 0;
    return avg;
  }

  unsigned int getSyncYFallingErrorCount() {
    unsigned int ec = cycleData[1].syncFallingErrorCount;
    cycleData[1].syncFallingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepYRisingErrorCount() {
    unsigned int ec = cycleData[1].sweepRisingErrorCount;
    cycleData[1].sweepRisingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepYFallingErrorCount() {
    unsigned int ec = cycleData[1].sweepFallingErrorCount;
    cycleData[1].sweepFallingErrorCount = 0;
    return ec;
  }

  unsigned int getSweepYAverage() {
    unsigned int avg = (unsigned int)(cycleData[1].sweepAccumulator / cycleData[1].sweepCounter);
    cycleData[1].sweepAccumulator = 0;
    cycleData[1].sweepCounter = 0;
    return avg;
  }
#endif

  unsigned long getXSyncTickCount() { return cycleData[0].syncTickCount; }
  unsigned long getXSweepTickCount() { return cycleData[0].sweepTickCount; }
  
  unsigned long getYSyncTickCount() { return cycleData[1].syncTickCount; }
  unsigned long getYSweepTickCount() { return cycleData[1].sweepTickCount; }

  //info about the robot position; if any portion of each Lighthouse cycle is missed, hasPosition() returns false
  KVector2* getPosition() { return &positionVector; }
  
  //data derived from change in position over time
  double getVelocity() { return velocity; }

};

class Lighthouse
{

private:
  //sensor above the left wheel
  LighthouseSensor leftSensor;
  //sensor above the right wheel
  LighthouseSensor rightSensor;
  
  KVector2 previousOrientationVector;
  unsigned long previousOrientationTimeStamp = 0;

  KVector2 orientationVector;
  unsigned long orientationTimeStamp = 0;

  //overall position, which is an average of both sensor positions
  KVector2 previousPositionVector;
  unsigned long previousPositionTimeStamp = 0;

  //overall position, which is an average of both sensor positions
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  void setupClock();
  void setupEIC();
  void connectPortPinsToInterrupts();
  void connectInterruptsToTimer();
  void setupTimer();

public:
  Lighthouse();

  void start();
  void loop();

  bool hasLighthouseSignal() { return leftSensor.hasLighthouseSignal() && rightSensor.hasLighthouseSignal(); }
  void recalculate();
  LighthouseSensor* getLeftSensor() { return &leftSensor; }
  LighthouseSensor* getRightSensor() { return &rightSensor; }

  KVector2* getPosition() { return &positionVector; }
  KVector2* getOrientation() { return &orientationVector; }
  double getRotationalVelocity();
  
  void stop();
  
};


