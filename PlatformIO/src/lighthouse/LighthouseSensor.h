
#ifndef _LIGHTHOUSESENSOR_H_
#define _LIGHTHOUSESENSOR_H_

#include <Arduino.h>
#include "KVector2.h"
#include "KQuaternion3.h"
#include "LighthouseOOTX.h"

#define BUFFER_SIZE 128
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

enum PulseCaptureState
{
    Unknown, SyncPulse, SweepStart, SweepHitPulse, SweepEnd
};

typedef struct _LighthouseSensorInput
{
  unsigned int hitTickBuffer[BUFFER_SIZE];
  unsigned int* const hitTickEndPtr = hitTickBuffer + BUFFER_SIZE - 1;
  unsigned int* volatile hitTickWritePtr = hitTickBuffer;
  unsigned int* volatile hitTickReadPtr = hitTickEndPtr;
} LighthouseSensorInput;

typedef struct _LighthouseSensorAxis
{
  //number of ticks from the end of the sync signal to the start of the sweep hit
  unsigned long pendingSyncTicks = 0;
  unsigned long pendingSweepStartTicks = 0;
  unsigned long pendingSweepHitTicks = 0;
  unsigned long pendingSweepEndTicks = 0;

  //total number of ticks from the start of the sync signal to the start of the sweep hit
  unsigned long sweepHitTicks = 0;
  //updated each time a sweep hit is detected; set to zero when we fail to detect the lighthouse sweep
  unsigned long sweepHitTimeStamp = 0;
} LighthouseSensorAxis;

class LighthouseSensor
{

private:
  //for debugging
  int debugNumber;

  //raw input shared with the interrupt handler; interrupt handler writes and LighthouseSensor reads and processes
  LighthouseSensorInput* sensorInput;

  //captured data in each cycle for the x and y axes
  //x = 0; y = 1
  LighthouseSensorAxis cycleData[2];
  //the current axis we're processing
  int currentAxis = 0;
  //the current edge we're expecting within the current cycle
  PulseCaptureState currentCaptureState = Unknown;

  unsigned long detectedHitX = 0;
  unsigned long detectedHitZ = 0;
  unsigned long detectedHitTimeStamp = 0;

  //data and code required to process the OOTX frame, for the purpose of extracting the lighthouse orientation
  BaseStationInfoBlock baseStationInfoBlock[BASE_STATION_INFO_BLOCK_SIZE];
  int zeroCount;
  int syncBitCounter;
  unsigned short payloadLength;
  unsigned short payloadReadMask;
  int readInfoBlockIndex;
  byte readInfoBlockMask;
  bool preambleFound = false;

  //once the OOTX frame has been found and processed, the lighthouse orientation is calculated
  RotorFactoryCalibrationData xRotor;
  RotorFactoryCalibrationData zRotor;
  KQuaternion3 lighthouseOrientation;
  bool receivedLighthousePosition = false;

  unsigned int previousTickCount = 0;

  //position
  KVector2 positionVector;
  unsigned long positionTimeStamp = 0;

  void processOOTXBit(unsigned int syncDelta);
  void calculateLighthousePosition();

  void reacquireSyncPulse(unsigned int deltaTickCount);
  void processSyncPulse(unsigned int deltaTickCount);
  void captureSyncPulse(unsigned int deltaTickCount);
  void processSweepStart(unsigned long currentTime, unsigned int deltaTickCount);
  void processSweepHitPulse(unsigned int deltaTickCount);
  void processSweepEnd(unsigned int deltaTickCount);

  bool recalculate();

  void clearPreambleFlag() { preambleFound = false; }
  bool foundPreamble() const { return preambleFound; }

  int8_t getAccelDirX();
  int8_t getAccelDirY();
  int8_t getAccelDirZ();

  unsigned long getPendingPositionTimeStamp();

  void restart();

  friend class Lighthouse;

public:
  LighthouseSensor(LighthouseSensorInput* sensorInput, int debugNumber);

  bool loop(unsigned long currentTime);

  unsigned long getXSyncTickCount() const { return cycleData[0].pendingSyncTicks; }
  unsigned long getXSweepTickCount() const { return cycleData[0].sweepHitTicks; }
  unsigned long getYSyncTickCount() const { return cycleData[1].pendingSyncTicks; }
  unsigned long getYSweepTickCount() const { return cycleData[1].sweepHitTicks; }
  const KVector2* getPosition() const { return &positionVector; }

  bool isSignalLocked() { return receivedLighthousePosition && cycleData[0].sweepHitTimeStamp && cycleData[1].sweepHitTimeStamp; }

};

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks);
float float16ToFloat32(uint16_t half);

#endif
