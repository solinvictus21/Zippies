
#ifndef _SENSORFUSOR_H_
#define _SENSORFUSOR_H_

#include <Arduino.h>

#include "LighthouseSensor.h"
#include "LighthouseOOTX.h"
#include "zippies/ZippyMath.h"
#include "zippies/hardware/ZippyFace.h"

//#define DEBUG_SIGNAL_EDGES 1
// #define SENSOR_COUNT      1
#define SENSOR_COUNT      2

typedef enum class _LighthouseSignalState
{
  AcquiringSyncSignal,
  ReceivingLighthouseData,
  AcquiringSensorHits,
  AwaitingPositionLock,
  SignalLocked
} LighthouseSignalState;

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

class SensorFusor
{

private:
  ZippyFace display;

  //sensors
  LighthouseSensor sensors[SENSOR_COUNT];
  unsigned long fusedSyncPulses[2];
  LighthouseSignalState currentSignalState;
  unsigned long cycleProcessedTime = 0;

  //initialization of lighthouse sensing requires first downloading the base station info block and
  //using that information to calculate a quaternion that represents the lighthouse orientation
  LighthouseOOTX ootxParser;
  RotorFactoryCalibrationData xRotor;
  RotorFactoryCalibrationData zRotor;
  KQuaternion3 lighthouseOrientation;
  bool receivedLighthouseData = false;

  //after initialization, we can begin calculating the positions of each sensor
  KVector2 sensorPositions[2];

  //then we can use the sensor position to calculate the position; by maintaining the previous position
  //during each iteration, we can use the difference between each position to calculate the velocity
  // KMatrix2 previousPosition;
  // unsigned long previousPositionTimeStamp = 0;
  KMatrix2 position;
  unsigned long positionTimeStamp = 0;
  // KMatrix2 positionDelta;

  //once we have data for the previous position as well as the current position, then we wait a predetermined
  //period of time to let the position data "settle" such as, for example, when the user is still placing
  //the Zippy on the ground but the sensors have already started detecting hits from the lighthouse before
  //motion caused by the handler has fully completed
  unsigned long positionLockedTimeStamp = 0;

  int preambleBitCount = 0;
  // bool preambleFound = false;

  void setupClock();
  void setupEIC();
  void connectPortPinsToInterrupts();
  void connectInterruptsToTimer();
  void setupTimer();

  bool fuseSyncPulses();
  void calculateLighthouseData();
  void recalculatePosition(unsigned long currentTime);
  void calculateSensorPosition(unsigned long xTicks, unsigned long zTicks, KVector2* out);
  void calculatePosition();
  void processPreambleBit(unsigned long syncTickCount);
  // void estimatePosition(unsigned long currentTime);

public:
  SensorFusor();

  void start();
  bool loop(unsigned long currentTime);
  void syncWithPreamble() { preambleBitCount = 0; }
  bool foundPreamble() const { return preambleBitCount == 17; }

  const LighthouseSensor* getLeftSensor() const { return &sensors[0]; }
  const LighthouseSensor* getRightSensor() const { return &sensors[1]; }

  unsigned long getPositionTimeStamp() const { return positionTimeStamp; }
  const KMatrix2* getPosition() const { return &position; }
  // const KMatrix2* getPositionDelta() const { return &positionDelta; }

  void stop();

};

#endif
