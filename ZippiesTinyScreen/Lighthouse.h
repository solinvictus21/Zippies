
#pragma once

#include <Arduino.h>
#include "KVector.h"
#include "KQuaternion.h"

//#define DEBUG_SIGNAL_EDGES 1
//#define DEBUG_LIGHTHOUSE_EDGES 1
//#define DEBUG_OOTX 1
//#define DEBUG_OOTX_ERRPRS 1

#define BUFFER_SIZE 32
#define BASE_STATION_INFO_BLOCK_SIZE 33

typedef struct _RotorFactoryCalibrationData
{
  //positive phase indicates that the beam is ahead of the ideal, which means that it will strike the sensors before we think it
  //should, leading us to believe that the angle is smaller than it is; thus we must add the phase for each rotor to the angle to
  double phase = 0.0d;
  double curve = 0.0d;
  double tilt = 0.0d;
  //gibbous magnitude indicates how much faster or slower the beam moves through the cycle than we expect; gibbous phase indicates
  //the center point in radians around which the magnitude scales the speed of the beam; so when gibbous phase is zero, the gibbous
  //magnitude scales the speed of the beam around the ideal center point and enters and leaves the field of view at equal offsets
  //at both the start and end of the cycle; positive gibbous phase indicates that the laser matches the ideal after the center
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

typedef struct _LighthouseSensorAxis
{
  //pending sync tick count; to be used if we later see the sweep hit
  unsigned long pendingCycleStart = 0;
  
  //number of ticks from the start of the visible portion of the cycle to the sweep hit
  unsigned long sweepTickCount = 0;
  //updated each time a sweep hit is detected; set to zero when we fail to detect the lighthouse sweep
  unsigned long sweepHitTimeStamp = 0;

#ifdef DEBUG_LIGHTHOUSE_EDGES
  //useful debugging statistics
  unsigned int syncHits = 0;
  unsigned int syncMisses = 0;
  unsigned long syncAccumulator = 0;
  unsigned long syncCounter = 0;
  unsigned int sweepHits = 0;
  unsigned int sweepMisses = 0;
  unsigned long sweepMinTicks = 0;
  unsigned long sweepMaxTicks = 0;
  unsigned long sweepAccumulator = 0;
  unsigned long sweepCounter = 0;
  //edges detected between the sweep hit and the next sync, which are "echoes" of other edges due to a likely circuit design shortcoming
  unsigned int edgeEchoCount = 0;
#endif
} LighthouseSensorAxis;

class LighthouseSensor
{

private:
  //for debugging
  int debugNumber;

  //raw input shared with the interrupt handler; interrupt handler writes and LighthouseSensor reads and processes
  LighthouseSensorInput* sensorInput;

  //the current edge we're expecting within the current cycle
  PulseEdge currentEdge;
  //the current axis we're processing
  int currentAxis;
  //captured data in each cycle for the x and y axes
  //x = 0; y = 1
  LighthouseSensorAxis cycleData[2];
  
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
  RotorFactoryCalibrationData zRotor;

  unsigned int previousTickCount;

  //historical data for calculating velocity
  KVector2 previousPositionVector;
  unsigned long previousPositionTimeStamp = 0;

  //position
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  double velocity;
  unsigned long velocityTimeStamp = 0;
  
  void reacquireSyncPulses(unsigned int previousTickCount, unsigned int currentTickCount);
  void processSyncRisingEdge(unsigned int previousTickCount, unsigned int currentTickCount);
  void processSyncFallingEdge(unsigned int previousTickCount, unsigned int currentTickCount);
  void captureSyncFallingEdge(unsigned int previousTickCount, unsigned int deltaTickCount);
  void processSweepRisingEdge(unsigned int previousTickCount, unsigned int currentTickCount);
  void processSweepFallingEdge(unsigned int previousTickCount, unsigned int currentTickCount);

  bool hasLighthouseSignal() { return receivedLighthousePosition && cycleData[0].sweepHitTimeStamp && cycleData[1].sweepHitTimeStamp; }
  void recalculatePosition();
  void estimatePosition(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long currentTime);
  void recalculateVelocity(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long orientationTimeStamp);

  friend class Lighthouse;

public:
  LighthouseSensor(LighthouseSensorInput* sensorInput, int debugNumber);

  void loop();
  
  //info about the lighthouse position received by this sensor
  bool hasLighthousePosition() { return receivedLighthousePosition; }
  int8_t getAccelDirX();
  int8_t getAccelDirY();
  int8_t getAccelDirZ();

  unsigned long getXSweepTickCount() { return cycleData[0].sweepTickCount; }
  
  unsigned long getYSweepTickCount() { return cycleData[1].sweepTickCount; }

  //info about the robot position; if any portion of each Lighthouse cycle is missed, hasPosition() returns false
  KVector2* getPosition() { return &positionVector; }
  
  //data derived from change in position over time
  double getVelocity() { return velocity; }

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
  unsigned int getXSyncHits() {
    unsigned int v = cycleData[0].syncHits;
    cycleData[0].syncHits = 0;
    return v;
  }

  unsigned int getXSyncMisses() {
    unsigned int v = cycleData[0].syncMisses;
    cycleData[0].syncMisses = 0;
    return v;
  }

  unsigned int getXSyncTicksAverage() {
    unsigned int v = cycleData[0].syncAccumulator / cycleData[0].syncCounter;
    cycleData[0].syncAccumulator = 0;
    cycleData[0].syncCounter = 0;
    return v;
  }
  
  unsigned int getXSweepHits() {
    unsigned int v = cycleData[0].sweepHits;
    cycleData[0].sweepHits = 0;
    return v;
  }

  unsigned int getXSweepMisses() {
    unsigned int v = cycleData[0].sweepMisses;
    cycleData[0].sweepMisses = 0;
    return v;
  }
  
  unsigned int getXSweepMinTicks() {
    unsigned int v = cycleData[0].sweepMinTicks;
    cycleData[0].sweepMinTicks = 0;
    return v;
  }
  
  unsigned int getXSweepMaxTicks() {
    unsigned int v = cycleData[0].sweepMaxTicks;
    cycleData[0].sweepMaxTicks = 0;
    return v;
  }
  
  unsigned int getXSweepTicksAverage() {
    unsigned int v = cycleData[0].sweepAccumulator / cycleData[0].sweepCounter;
    cycleData[0].sweepAccumulator = 0;
    cycleData[0].sweepCounter = 0;
    return v;
  }
  
  unsigned int getXEdgeEchoCount() {
    unsigned int v = cycleData[0].edgeEchoCount;
    cycleData[0].edgeEchoCount = 0;
    return v;
  }
  
  unsigned int getYSyncHits() {
    unsigned int v = cycleData[1].syncHits;
    cycleData[1].syncHits = 0;
    return v;
  }

  unsigned int getYSyncMisses() {
    unsigned int v = cycleData[1].syncMisses;
    cycleData[1].syncMisses = 0;
    return v;
  }

  unsigned int getYSyncTicksAverage() {
    unsigned int v = cycleData[1].syncAccumulator / cycleData[1].syncCounter;
    cycleData[1].syncAccumulator = 0;
    cycleData[1].syncCounter = 0;
    return v;
  }
  
  unsigned int getYSweepHits() {
    unsigned int v = cycleData[1].sweepHits;
    cycleData[1].sweepHits = 0;
    return v;
  }

  unsigned int getYSweepMisses() {
    unsigned int v = cycleData[1].sweepMisses;
    cycleData[1].sweepMisses = 0;
    return v;
  }

  unsigned int getYSweepMinTicks() {
    unsigned int v = cycleData[1].sweepMinTicks;
    cycleData[1].sweepMinTicks = 0;
    return v;
  }
  
  unsigned int getYSweepMaxTicks() {
    unsigned int v = cycleData[1].sweepMaxTicks;
    cycleData[1].sweepMaxTicks = 0;
    return v;
  }
  
  unsigned int getYSweepTicksAverage() {
    unsigned int v = cycleData[1].sweepAccumulator / cycleData[1].sweepCounter;
    cycleData[1].sweepAccumulator = 0;
    cycleData[1].sweepCounter = 0;
    return v;
  }
  
  unsigned int getYEdgeEchoCount() {
    unsigned int v = cycleData[1].edgeEchoCount;
    cycleData[1].edgeEchoCount = 0;
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
  double getLinearVelocity() { return (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2.0d; }
  KVector2* getOrientation() { return &orientationVector; }
  
  void stop();
  
};


