
#include "LighthouseSensor.h"
#include "KVector3.h"
#include "../ZippyConfig.h"

//#define LIGHTHOUSE_DEBUG_ERRORS 1
// #define DEBUG_LIGHTHOUSE_EDGES 1
// #define DEBUG_OOTX 1
//#define DEBUG_OOTX_ERRPRS 1

//height of the lighthouse from the floor
//mounted on surface of entertainment center
#define LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM 930.0d
//mounted on top of TV
// #define LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM            1920.0d
//the Z axis orientation offset (30 degrees)
// #define LIGHTHOUSE_ORIENTATION_Z                             0.523598775598299d

//height of the diode sensors from the floor
#define ROBOT_DIODE_HEIGHT_MM 42.0d

//timings for 48 MHz
//use the full 180 degrees to determine the actual range of ticks
#define SWEEP_DURATION_TICK_COUNT 400000
//this is the portion of the total sweep cycle duration during which we will reject echoes and reflections from the incoming signal
#define SWEEP_DURATION_EPSILON      3000
//x axis, OOTX bit 0
#define SYNC_PULSE_J0_MIN 2945
//y axis, OOTX bit 0
#define SYNC_PULSE_K0_MIN 3445
//x axis, OOTX bit 1
#define SYNC_PULSE_J1_MIN 3945
//y axis, OOTX bit 1
#define SYNC_PULSE_K1_MIN 4445
#define NONSYNC_PULSE_J2_MIN 4950
#define SWEEP_HIT_MAX_DURATION      1500

LighthouseSensor::LighthouseSensor(LighthouseSensorInput* sensorInput,
                                   int dn)
  : debugNumber(dn),
    zeroCount(0),
    syncBitCounter(0),
    payloadLength(0),
    payloadReadMask(0),
    readInfoBlockIndex(0),
    readInfoBlockMask(0)
{
  this->sensorInput = sensorInput;
}

void LighthouseSensor::restart()
{
  sensorInput->hitTickWritePtr = sensorInput->hitTickBuffer;
  sensorInput->hitTickReadPtr = sensorInput->hitTickEndPtr;

  memset(cycleData, 0, sizeof(cycleData));
  currentAxis = 0;
  currentCaptureState = Unknown;

  detectedHitX = 0;
  detectedHitZ = 0;
  detectedHitTimeStamp = 0;

  zeroCount = 0;
  syncBitCounter = 0;
  payloadLength = 0;
  payloadReadMask = 0;
  readInfoBlockIndex = 0;
  readInfoBlockMask = 0;
  preambleFound = false;

  receivedLighthousePosition = false;

  previousTickCount = 0;

  positionTimeStamp = 0;
}


int8_t LighthouseSensor::getAccelDirX() {
  return ((BaseStationInfoBlock*)baseStationInfoBlock)->accel_dir_x;
}

int8_t LighthouseSensor::getAccelDirY() {
  return ((BaseStationInfoBlock*)baseStationInfoBlock)->accel_dir_y;
}

int8_t LighthouseSensor::getAccelDirZ() {
  return ((BaseStationInfoBlock*)baseStationInfoBlock)->accel_dir_z;
}

bool LighthouseSensor::loop(unsigned long currentTime)
{
#ifdef LIGHTHOUSE_DEBUG_ERRORS
  //ensure we didn't overflow our buffer; show a warning if so
  if (sensorInput->hitTickWritePtr == sensorInput->hitTickReadPtr) {
    SerialUSB.print(debugNumber);
    SerialUSB.println(" WARNING: Buffer overflow. Potential missed frames.");
  }
#endif

  //this is volatile, so grab it first
  unsigned int* hitTickWritePtr = (unsigned int*)sensorInput->hitTickWritePtr;
//  int edgeCount = 0;
//  SerialUSB.println("Processing sensor.");
  while (true) {
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

//    edgeCount++;
    //then let the read pointer move forward; must be atomic, hence the temp pointer above until we confirmed we can move forward
    sensorInput->hitTickReadPtr = nextReadPtr;

    //get our next tick count delta
    unsigned int currentTickCount = *nextReadPtr;
    unsigned int deltaTickCount = calculateDeltaTicks(previousTickCount, currentTickCount);
    /*
    SerialUSB.print("Next edge: ");
    SerialUSB.print(previousTickCount);
    SerialUSB.print(" ");
    SerialUSB.print(currentTickCount);
    SerialUSB.print(" ");
    SerialUSB.println(deltaTickCount);
    */
    previousTickCount = currentTickCount;

    switch (currentCaptureState) {
      case Unknown:
        reacquireSyncPulse(deltaTickCount);
        break;

      case SyncPulse:
        processSyncPulse(deltaTickCount);
        break;

      case SweepStart:
        processSweepStart(currentTime, deltaTickCount);
        break;

      case SweepHitPulse:
        processSweepHitPulse(deltaTickCount);
        break;

      case SweepEnd:
        processSweepEnd(deltaTickCount);
        break;
    }
  }

/*
  if (edgeCount > 10) {
    SerialUSB.print("Processed edges: ");
    SerialUSB.println(edgeCount);
  }
*/

  return receivedLighthousePosition && detectedHitTimeStamp > positionTimeStamp;
}

void LighthouseSensor::reacquireSyncPulse(unsigned int deltaTickCount)
{
  if (deltaTickCount < SYNC_PULSE_J0_MIN || deltaTickCount >= NONSYNC_PULSE_J2_MIN)
    return;

//  SerialUSB.println("Acquired sync pulse");
  //found a sync signal; capture the sync data and go back to lock-step tracking
  currentAxis = ((deltaTickCount - SYNC_PULSE_J0_MIN) / 500) & 0x1;
  captureSyncPulse(deltaTickCount);
}

void LighthouseSensor::processSyncPulse(unsigned int deltaTickCount)
{
  if (deltaTickCount < SYNC_PULSE_J0_MIN || deltaTickCount >= NONSYNC_PULSE_J2_MIN) {
    //we missed an expected sync signal
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Missed falling sync: ");
      SerialUSB.println(deltaTickCount);
    }
    //this technically counts as two errors, since we will now also miss the sweep pulse
//    cycleData[currentAxis].syncMisses++;
//    cycleData[currentAxis].sweepMisses++;
#endif

    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;
    currentCaptureState = Unknown;
    return;
  }

  //sanity check to ensure we're receiving the sync signal for the axis that we're expecting
  int newAxis = ((deltaTickCount - SYNC_PULSE_J0_MIN) / 500) & 0x1;
  if (newAxis != currentAxis) {
    //wrong sync pulse; this is for the other axis
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Got wrong sync: ");
      SerialUSB.println(deltaTickCount);
    }
    //this technically counts as two errors, since we also mised the sweep pulse for this axes
//    cycleData[currentAxis].syncMisses++;
//    cycleData[currentAxis].sweepMisses++;
#endif

    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;

    //now switch to the correct axis
    currentAxis = newAxis;
  }

  captureSyncPulse(deltaTickCount);
}

void LighthouseSensor::captureSyncPulse(unsigned int deltaTickCount)
{
  //this is the falling edge for the sync pulse along the current axis
  if (!receivedLighthousePosition)
    processOOTXBit(deltaTickCount);
  else if (!preambleFound) {
    if (deltaTickCount < SYNC_PULSE_J1_MIN) {
      zeroCount++;
      if (zeroCount == 17)
        preambleFound = true;
    }
    else
      zeroCount = 0;
  }

  cycleData[currentAxis].pendingSyncTicks = deltaTickCount;
  currentCaptureState = SweepStart;
}

void LighthouseSensor::processSweepStart(unsigned long currentTime, unsigned int deltaTickCount)
{
  //this should be the rising edge of the sweep hit
  unsigned long totalSweepLength = cycleData[currentAxis].pendingSyncTicks + deltaTickCount;
  if (totalSweepLength >= SWEEP_DURATION_TICK_COUNT - SWEEP_DURATION_EPSILON) {
    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;

    //move to watching for the end of the sync pulse on the other axis
    if (totalSweepLength >= SWEEP_DURATION_TICK_COUNT + SWEEP_DURATION_EPSILON) {
      //we missed the sweep hit for this axis, and the total sweep length is long enough that we now
      //have no idea which signal we'll receive next
#ifdef DEBUG_LIGHTHOUSE_EDGES
      if (debugNumber == 0) {
        SerialUSB.print("Lost signal lock: ");
        SerialUSB.println(totalSweepLength);
      }
//      cycleData[currentAxis].sweepMisses++;
#endif
      currentCaptureState = Unknown;
      return;
    }

    //we missed the sweep hit for this axis, but we're within the window to hit the next sync signal
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Missed ");
      SerialUSB.print(!currentAxis ? "X" : "Y");
      SerialUSB.print(" sweep hit: ");
      SerialUSB.print(cycleData[currentAxis].pendingSyncTicks);
      SerialUSB.print(" ");
      SerialUSB.print(deltaTickCount);
      SerialUSB.print(" ");
      SerialUSB.println(totalSweepLength);
    }
//    cycleData[currentAxis].sweepMisses++;
#endif

    currentAxis = (currentAxis+1) & 0x1;
    currentCaptureState = SyncPulse;
    return;
  }

  //this sweep hit is hitting within the expected sweep window; wait for the falling edge
  cycleData[currentAxis].pendingSweepStartTicks = deltaTickCount;
  cycleData[currentAxis].sweepHitTicks = cycleData[currentAxis].pendingSyncTicks +
      cycleData[currentAxis].pendingSweepStartTicks;
  cycleData[currentAxis].sweepHitTimeStamp = currentTime;
  currentCaptureState = SweepHitPulse;

  if (currentAxis == 1 && cycleData[0].sweepHitTimeStamp) {
    //commit the combined sweep data to be processed for location calculation
    detectedHitX = cycleData[0].sweepHitTicks;
    detectedHitZ = cycleData[1].sweepHitTicks;
    detectedHitTimeStamp = max(cycleData[0].sweepHitTimeStamp, cycleData[1].sweepHitTimeStamp);
  }
}

void LighthouseSensor::processSweepHitPulse(unsigned int deltaTickCount)
{
  if (deltaTickCount >= SWEEP_HIT_MAX_DURATION) {
#ifdef DEBUG_LIGHTHOUSE_EDGES
    if (debugNumber == 0) {
      SerialUSB.print("Missed sweep hit falling edge: ");
      SerialUSB.println(deltaTickCount);
    }
#endif
    cycleData[currentAxis].pendingSweepHitTicks = 0;
//    cycleData[currentAxis].pendingSweepEndTicks = 0;
//    processSweepEnd(deltaTickCount - SWEEP_HIT_MAX_DURATION);
    processSweepEnd(deltaTickCount);
    return;
  }

  cycleData[currentAxis].pendingSweepHitTicks = deltaTickCount;
//  cycleData[currentAxis].pendingSweepEndTicks = 0;
  currentCaptureState = SweepEnd;
}

void LighthouseSensor::processSweepEnd(unsigned int deltaTickCount)
{
// SerialUSB.print("Sweep end: ");
// SerialUSB.println(deltaTickCount);
  cycleData[currentAxis].pendingSweepEndTicks = deltaTickCount;
  unsigned long totalCycleLength = cycleData[currentAxis].pendingSyncTicks +
      cycleData[currentAxis].pendingSweepStartTicks +
      cycleData[currentAxis].pendingSweepHitTicks +
      cycleData[currentAxis].pendingSweepEndTicks;
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
      SerialUSB.print(cycleData[currentAxis].pendingSyncTicks);
      SerialUSB.print(" ");
      SerialUSB.print(cycleData[currentAxis].pendingSweepStartTicks);
      SerialUSB.print(" ");
      SerialUSB.print(cycleData[currentAxis].pendingSweepHitTicks);
      SerialUSB.print(" ");
      SerialUSB.print(cycleData[currentAxis].pendingSweepEndTicks);
      SerialUSB.print(" ");
      SerialUSB.println(totalCycleLength);
    }
#endif

    cycleData[currentAxis].sweepHitTimeStamp = 0;
    currentCaptureState = Unknown;
    return;
  }

  //found rising edge of the sync pulse
  currentAxis = (currentAxis+1) & 0x1;
  currentCaptureState = SyncPulse;
}

/**
   Each sync pulse represents either a zero bit (3000-4000 ticks pulse width) or a one bit (4000-5000) of the OOTX
   frame, with a one bit always occurring every 17th pulse to frame the data bits (aka, the "sync bit"), and 17 zero
   bits representing the start of the frame, since 17 zero bits cannot occur in the middle of the data stream due
   to the sync bits.
*/
void LighthouseSensor::processOOTXBit(unsigned int syncDelta)
{
  bool value = syncDelta >= SYNC_PULSE_J1_MIN;
/*
#ifdef DEBUG_OOTX
  SerialUSB.print(debugNumber);
  SerialUSB.print(" Received an OOTX bit. Ticks: ");
  SerialUSB.print(syncDelta);
  SerialUSB.print("   Bit: ");
  SerialUSB.println(value ? 1 : 0);
#endif
*/

  syncBitCounter++;
  if (!value) {
    zeroCount++;

    if (zeroCount == 17) {
      //found start of OOTX frame
      preambleFound = true;
#ifdef DEBUG_OOTX
      SerialUSB.print(debugNumber);
      SerialUSB.println(" Found the start of the OOTX frame.");
#endif

#ifdef DEBUG_OOTX_ERRPRS
      if (payloadReadMask || readInfoBlockMask) {
        SerialUSB.print(debugNumber);
        SerialUSB.println(" WARNING: OOTX frame restarted");
      }
#endif
      //cancel any packet we were previously reading
      readInfoBlockMask = 0;

      zeroCount = 0;
      syncBitCounter = 16;
      payloadReadMask = 0x0080;

      return;
    }
    else if (syncBitCounter == 17 && (payloadReadMask || readInfoBlockMask)) {
      //expecting a sync bit and didn't get it; start over
#ifdef DEBUG_OOTX_ERRPRS
      SerialUSB.print(debugNumber);
      SerialUSB.print(" WARNING: Missed a sync bit: ");
      SerialUSB.println(syncDelta);
#endif
      syncBitCounter = 0;

      //cancel anything we were previously reading; wait for the start of another OOTX frame
      payloadReadMask = 0;
      readInfoBlockMask = 0;
      return;
    }
  }
  else {
    zeroCount = 0;
    if (syncBitCounter == 17) {
      //excellent; got a sync bit right where we expected it
      syncBitCounter = 0;
      //now is this the end of the info block
      if (readInfoBlockIndex == BASE_STATION_INFO_BLOCK_SIZE) {
        readInfoBlockIndex = 0;

#ifdef DEBUG_OOTX
        SerialUSB.print(debugNumber);
        SerialUSB.println(" Got the base station info block.");
#endif

        //now calculate the position and orientation of the lighthouse
        calculateLighthousePosition();
      }
      return;
    }
  }

  if (payloadReadMask) {
    if (value)
      payloadLength |= payloadReadMask;
    else
      payloadLength &= ~payloadReadMask;

    payloadReadMask >>= 1;
    if (payloadReadMask == 0) {
      payloadReadMask = 0x8000;
    }
    else if (payloadReadMask == 0x0080) {
      payloadReadMask = 0;
      if (payloadLength == BASE_STATION_INFO_BLOCK_SIZE) {
#ifdef DEBUG_OOTX
        SerialUSB.print(debugNumber);
        SerialUSB.println(" Starting to read base station info block.");
#endif
        readInfoBlockIndex = 0;
        readInfoBlockMask = 0x80;
      }
#ifdef DEBUG_OOTX_ERRPRS
      else {
        SerialUSB.print(debugNumber);
        SerialUSB.print(" WARNING: Receiving an OOTX frame that is NOT the base station info block of size: ");
        SerialUSB.println(payloadLength, BIN);
      }
#endif
    }
  }
  else if (readInfoBlockMask) {
    if (value)
      ((uint8_t*)baseStationInfoBlock)[readInfoBlockIndex] |= readInfoBlockMask;
    else
      ((uint8_t*)baseStationInfoBlock)[readInfoBlockIndex] &= ~readInfoBlockMask;

    readInfoBlockMask >>= 1;
    if (readInfoBlockMask == 0) {
      readInfoBlockIndex++;
      if (readInfoBlockIndex < BASE_STATION_INFO_BLOCK_SIZE)
        readInfoBlockMask = 0x80;
    }
  }
}

/**
 * Calculate the orientation and position of the lighthouse.
 *
 * From the accelerometer reading, calculate a quaternion that represents the lighthouse rotation in a coordinate system where the
 * x and y axes are parallel to the ground, positive x is to the right from the lighthouse, positive y is forward from the lighthouse,
 * and positive z represents height.
 */
void LighthouseSensor::calculateLighthousePosition()
{
  //capture the factory calibration data for the x rotor
  xRotor.phase = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_0_phase);
  xRotor.tilt = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_0_tilt);
  xRotor.curve = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_0_curve);
  xRotor.gibbousPhase = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_0_gibphase);
  xRotor.gibbousMagnitude = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_0_gibmag);

  //capture the factory calibration data for the z rotor
  zRotor.phase = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_1_phase);
  zRotor.tilt = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_1_tilt);
  zRotor.curve = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_1_curve);
  zRotor.gibbousPhase = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_1_gibphase);
  zRotor.gibbousMagnitude = float16ToFloat32(((BaseStationInfoBlock*)baseStationInfoBlock)->fcal_1_gibmag);

#ifdef DEBUG_OOTX
  SerialUSB.println("X Rotor Factory Calibration:");
  SerialUSB.println((xRotor.phase / M_PI) * 180.0d, 6);
  SerialUSB.println((xRotor.tilt / M_PI) * 180.0d, 6);
  SerialUSB.println((xRotor.curve / M_PI) * 180.0d, 6);
  SerialUSB.println(xRotor.gibbousPhase, 6);
  SerialUSB.println(xRotor.gibbousMagnitude, 6);
  SerialUSB.println();

  SerialUSB.println("Z Rotor Factory Calibration:");
  SerialUSB.println((zRotor.phase / M_PI) * 180.0d, 6);
  SerialUSB.println((zRotor.tilt / M_PI) * 180.0d, 6);
  SerialUSB.println((zRotor.curve / M_PI) * 180.0d, 6);
  SerialUSB.println(zRotor.gibbousPhase, 6);
  SerialUSB.println(zRotor.gibbousMagnitude, 6);
  SerialUSB.println();

  // SerialUSB.println(getAccelDirX(), 6);
  // SerialUSB.println(getAccelDirY(), 6);
  // SerialUSB.println(getAccelDirZ(), 6);
#endif

  // /*
  //The accelerometer reading from the lighthouse gives us a vector that represents the lighthouse "up" direction in a coordinate system
  //where the x and z axes are parallel to the ground, positive x is to the lighthouse "left", positive z is "forward, and positive y is
  //"up". This means swapping the y and z axes of the accelerometer and flipping the x axis to put them into our global coordinate system.
  KVector3 rotationUnitVector(-getAccelDirX(), getAccelDirZ(), getAccelDirY(), 1.0d);

  //now calculate the angle of rotation from the "up" normal in our global coordinate system (0,0,1) to the rotation unit vector
  //this calculation ultimately reduces to the inverse cosine of the z axis of the rotation unit vector
  double angleOfRotation = acos(rotationUnitVector.getZ());

  //now cross the "up" vector of the lighthouse with the "up" normal of the global coordinate system to obtain the axis of rotation for
  //our quaternion; this calculation ultimately reduces to the y axis from the rotation unit vector becoming the x axis and the x axis
  //becoming the negative y axis; then obtain the unit vector of the result
  rotationUnitVector.set(rotationUnitVector.getY(), -rotationUnitVector.getX(), 0.0d, 1.0d);
  // */

  //now that we have both the axis and angle of rotation, we can calculate our quaternion
  lighthouseOrientation.set(rotationUnitVector.getX(), rotationUnitVector.getY(), rotationUnitVector.getZ(), angleOfRotation);

#ifdef LIGHTHOUSE_ORIENTATION_Z
  lighthouseOrientation.rotateZ(LIGHTHOUSE_ORIENTATION_Z);
#endif

  receivedLighthousePosition = true;
}

/*
 * Translates combined x and y tick counts into a vector from the lighthouse to the zippy in the global coordinate system.
 */
bool LighthouseSensor::recalculate()
{
  if (!receivedLighthousePosition || detectedHitTimeStamp <= positionTimeStamp) {
    //position is already up-to-date
    /*
    if (receivedLighthousePosition) {
      SerialUSB.print(debugNumber);
      SerialUSB.print(" Failed to update position: ");
      SerialUSB.print(detectedHitTimeStamp);
      SerialUSB.print(" - ");
      SerialUSB.println(positionTimeStamp);
    }
    */
    return false;
  }

  //Step 1: Calculate the vector from the lighthouse in its reference frame to the diode by normalizing the angle on each axis
  //from the lighthouse to +/- M_PI_2
  double observedAngleX = ((((double)detectedHitX) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;
  double observedAngleZ = ((((double)detectedHitZ) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;

  /*
  elevation += phase[lighthouse][VERTICAL];
  elevation += curve[lighthouse][VERTICAL]*pow(sin(elevation)*cos(azimuth),2.0) + gibmag[lighthouse][VERTICAL]*cos(elevation+gibphase[lighthouse][VERTICAL]);
  azimuth += phase[lighthouse][HORIZONTAL];
  azimuth += curve[lighthouse][HORIZONTAL]*pow(cos(elevation),2.0) + gibmag[lighthouse][HORIZONTAL]*cos(azimuth+gibphase[lighthouse][HORIZONTAL]);
   */
  //correct for the factory calibration data; formula pulled from notes on the open source libsurvive project
  //    https://github.com/cnlohr/libsurvive/wiki/BSD-Calibration-Values
  //at y=1, we want the x and z coordinates of our direction vector; since TAN = O / A, then O = TAN / A and given that our adjacent
  //is 1.0, then the opposite is simply the TAN of the angle along each axis
  double idealAngleX = observedAngleX + xRotor.phase;
  double idealAngleZ = observedAngleZ + zRotor.phase;

  // /*
  double x = tan(idealAngleX);
  double z = tan(idealAngleZ);
  idealAngleX +=
      (tan(xRotor.tilt) * z) +
      (xRotor.curve * z * z) +
      (sin(xRotor.gibbousPhase + idealAngleX) * xRotor.gibbousMagnitude);
  idealAngleZ +=
      (tan(zRotor.tilt) * x) +
      (zRotor.curve * x * x) +
      (sin(zRotor.gibbousPhase + idealAngleZ) * zRotor.gibbousMagnitude);
  // */

  //calculate the normal for the plane of the intersection from the horizontal sweep, which is a vertical plane whose normal
  //is a vector with no z component
  //note that we must flip the x axis while converting to our global coordinate system because our tick counts get greater
  //from right-to-left from the perspective of the lighthouse; this is contrary to some animations online which illustrate
  //the horizontal beam sweeping from left-to-right from the perspective of the lighthouse
  double verticalNormalX = cos(-idealAngleX);
  double verticalNormalY = sin(idealAngleX);
  //calculate the normal for the plane of the intersection from the vertical sweep, which is a horizontal plane whose normal
  //is a vector with no x component
  double horizontalNormalY = -sin(idealAngleZ);
  double horizontalNormalZ = cos(idealAngleZ);
  //now cross the plane of the vertical sweep with the plane of the horizontal sweep and normalize it; this will give us a
  //unit vector which represents the intersection of these two planes in the local coordinate system of the lighthouse
  //this calculation reduces to the following
  KVector3 directionFromLighthouse(
      -(horizontalNormalZ * verticalNormalY),
      horizontalNormalZ * verticalNormalX,
      -(horizontalNormalY * verticalNormalX),
      1.0d);
  //now convert the vector from the lighthouse in its local coordinate system to our global coordinate system
  directionFromLighthouse.unrotate(&lighthouseOrientation);

  //now intersect with the plane of the diodes on the robot; since our diode plane is defined by the normal 0,0,1, and we have a
  //vector which identifies the position of the lighthouse from 0,0,0, the entire formula for our diode-plane intersection reduces
  //to the following
  double lighthouseDistanceFromDiodePlane = LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM - ROBOT_DIODE_HEIGHT_MM;
  double t = -lighthouseDistanceFromDiodePlane / directionFromLighthouse.getZ();
  positionVector.set(
      (directionFromLighthouse.getX() * t) - LIGHTHOUSE_CENTER_OFFSET_X,
      (directionFromLighthouse.getY() * t) - LIGHTHOUSE_CENTER_OFFSET_Y);
  positionTimeStamp = detectedHitTimeStamp;

  return true;
}

unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks)
{
  /*
  if (startTicks > endTicks) {
      SerialUSB.print("overflow: ");
      SerialUSB.print(startTicks);
      SerialUSB.print(" ");
      SerialUSB.print(endTicks);
      SerialUSB.print(" ");
      SerialUSB.println((0x01000000 - startTicks) + endTicks);
  }
//  */
  //calculate the delta between the ticks; they are derived from a 24-bit counter, so we must be cautious to check when it rolls over
  return startTicks > endTicks
    ? (0x01000000 - startTicks) + endTicks
    : endTicks - startTicks;
}

/**
 * Utility function to convert a 16-bit IEEE floating point number to a 32-bit IEEE floating point number, needed to parse the
 * Lighthouse base station info block OOTX packet.
 */
float float16ToFloat32(uint16_t half)
{
  union {
    uint32_t u;
    float f;
  } val;
  val.u = (half & 0x7fff) << 13 | (half & 0x8000) << 16;
  if ((half & 0x7c00) != 0x7c00)
    return val.f * 0x1p112;
  val.u |= 0x7f800000;
  return val.f;
}
