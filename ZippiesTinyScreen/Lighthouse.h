
#pragma once

#include <Arduino.h>
#include "KVector.h"
#include "KQuaternion.h"

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
  unsigned int hitTickBuffer[BUFFER_SIZE];
  unsigned int* const hitTickEndPtr = hitTickBuffer + BUFFER_SIZE - 1;
  unsigned int* volatile hitTickWritePtr = hitTickBuffer;
  unsigned int* volatile hitTickReadPtr = hitTickEndPtr;
} LighthouseSensorInput;

enum CycleEdge
{
    SyncRising, SyncFalling, SweepRising, SweepFalling
};

typedef struct _SensorCycleData
{
  unsigned long pendingSyncTickCount = 0;
  //number of ticks in most recent cycle; set to zero when a cycle is missed
  unsigned long syncTickCount = 0;
  //number of ticks from the start of the visible portion of the sweep (30 degrees) to the sweep hit
  //set to zero when a cycle is missed
  unsigned long sweepTickCount = 0;

  //updated each time a sweep hit is detected; set to zero when lighthouse signal is unavailable
  unsigned long sweepHitTimeStamp = 0;
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

  //current cycle is one of the following:
  //  -1 : unknown/reacquiring sync signal
  //   0 : x axis
  //   1 : y axis
  int currentCycle;
  //the current edge we're expecting within the current cycle
  CycleEdge pendingCycleEdge;
  unsigned int previousTickCount;
  //captured data in each cycle for the x and y axes
  SensorCycleData cycleData[2];

  //historical data for calculating velocity
  KVector2 previousPositionVector;
  unsigned long previousPositionTimeStamp = 0;

  //position
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  double velocity;
  unsigned long velocityTimeStamp = 0;
  
  void processSyncSignal(unsigned int previousTicks, unsigned int currentTicks);
  void processSweepHit(unsigned int previousTicks, unsigned int currentTicks);

  bool hasLighthouseSignal() { return receivedLighthousePosition && cycleData[0].sweepHitTimeStamp && cycleData[1].sweepHitTimeStamp; }
  void recalculatePosition();
  void estimatePosition(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long currentTime);
  void recalculateVelocity(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long orientationTimeStamp);

  friend class Lighthouse;
  
public:
  LighthouseSensor(LighthouseSensorInput* sensorInput, int debugNumber);

  void loop();

  //info about the lighthouse position received by this sensor
  int8_t getAccelDirX();
  int8_t getAccelDirY();
  int8_t getAccelDirZ();

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


