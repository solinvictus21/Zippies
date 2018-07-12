
#include "LighthouseSensor.h"

//height of the lighthouse from the floor
//mounted on surface of entertainment center
//#define LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM 930.0d
//mounted on top of TV
#define LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM 1940.0d
//height of the diode sensors from the floor
#define ROBOT_DIODE_HEIGHT_MM 42.0d

//timings for 48 MHz
//use the full 180 degrees to determine the actual range of ticks
#define SWEEP_DURATION_TICK_COUNT 400000
//this is the portion of the total sweep cycle duration during which we will reject echoes and reflections from the incoming signal
#define SWEEP_DURATION_LAMBDA     397000
//x axis, OOTX bit 0
#define SYNC_PULSE_J0_MIN 2945
//y axis, OOTX bit 0
#define SYNC_PULSE_K0_MIN 3445
//x axis, OOTX bit 1
#define SYNC_PULSE_J1_MIN 3945
//y axis, OOTX bit 1
#define SYNC_PULSE_K1_MIN 4445
#define NONSYNC_PULSE_J2_MIN 4950

/**
   Convert a 16-bit IEEE floating point number to a 32-bit IEEE floating point number.
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

LighthouseSensor::LighthouseSensor(LighthouseSensorInput* sensorInput,
                                   int dn)
  : debugNumber(dn),
    currentEdge(Unknown),
    currentAxis(0),
    zeroCount(0),
    syncBitCounter(0),
    payloadLength(0),
    payloadReadMask(0),
    readInfoBlockIndex(0),
    readInfoBlockMask(0),
    receivedLighthousePosition(false),
    previousTickCount(0)
{
  this->sensorInput = sensorInput;
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

/**
 * Calculate the orientation and position of the lighthouse relative to the ground plane.
 *
 * From this accelerometer reading, calculate a quaternion that represents the lighthouse rotation in a coordinate system where the
 * x and y axes are parallel to the ground, positive x is to the right from the lighthouse, positive y is forward from the lighthouse,
 * and positive z represents height.
 */
unsigned int calculateDeltaTicks(unsigned int startTicks, unsigned int endTicks)
{
  //calculate the delta between the ticks; they are derived from a 24-bit counter, so we must be cautious to check when it rolls over
  return startTicks > endTicks
    ? (0x01000000 - startTicks) + endTicks
    : endTicks - startTicks;
}

//returns true when the x or y tick counts are updated
void LighthouseSensor::loop()
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

    //then let the read pointer move forward; must be atomic, hence the temp pointer above until we confirmed we can move forward
    sensorInput->hitTickReadPtr = nextReadPtr;

    //get our next tick count delta
    unsigned int currentTickCount = *nextReadPtr;

    switch (currentEdge) {
      case Unknown:
        reacquireSyncPulses(previousTickCount, currentTickCount);
        break;

      case SyncRising:
        processSyncRisingEdge(previousTickCount, currentTickCount);
        break;

      case SyncFalling:
        processSyncFallingEdge(previousTickCount, currentTickCount);
        break;

      case SweepRising:
        processSweepRisingEdge(previousTickCount, currentTickCount);
        break;

      case SweepFalling:
        processSweepFallingEdge(previousTickCount, currentTickCount);
        break;
    }
    previousTickCount = currentTickCount;
  }
}

void LighthouseSensor::reacquireSyncPulses(unsigned int previousTickCount, unsigned int currentTickCount)
{
  unsigned int deltaTickCount = calculateDeltaTicks(previousTickCount, currentTickCount);
  if (deltaTickCount < SYNC_PULSE_J0_MIN || deltaTickCount >= NONSYNC_PULSE_J2_MIN)
    return;

  //found a sync signal; capture the sync data and go back to lock-step tracking
  currentAxis = ((deltaTickCount - SYNC_PULSE_J0_MIN) / 500) & 0x1;
  captureSyncFallingEdge(previousTickCount, deltaTickCount);
}

void LighthouseSensor::processSyncRisingEdge(unsigned int previousTickCount, unsigned int currentTickCount)
{
  //calculate the total tick count since the rising edge of the previous cycle
  unsigned long deltaTickCount = calculateDeltaTicks(cycleData[currentAxis].pendingCycleStart, currentTickCount);

  if (deltaTickCount < SWEEP_DURATION_LAMBDA) {
    //the next rising pulse edge was detected before it should have been due to what appear to be shortcomings in the
    //current build of the lighthouse circuit; reject these pulses as "echoes"
    //in this particular case, the 2.2pF capacitors during transimpedance amplification appear to be generating the pulse
    //echoes; the next circuit build will use 5pF capacitors, which eliminates the pulse echoes between the falling edge
    //of the last sweep hit and the rising edge of the next sync pulse without reducing overall circuit sensitivity
#ifdef DEBUG_LIGHTHOUSE_EDGES
    cycleData[currentAxis].edgeEchoCount++;
#endif
    return;
  }

  //found rising edge of the sync pulse
  currentAxis = (currentAxis+1) & 0x1;
  currentEdge = SyncFalling;
}

void LighthouseSensor::processSyncFallingEdge(unsigned int previousTickCount, unsigned int currentTickCount)
{
  unsigned int deltaTickCount = calculateDeltaTicks(previousTickCount, currentTickCount);
  if (deltaTickCount < SYNC_PULSE_J0_MIN || deltaTickCount >= NONSYNC_PULSE_J2_MIN) {
    //we missed an expected sync signal
#ifdef DEBUG_LIGHTHOUSE_EDGES
    //this technically counts as two errors, since we will now also miss the sweep pulse
    cycleData[currentAxis].syncMisses++;
    cycleData[currentAxis].sweepMisses++;
#endif

    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;

    currentEdge = Unknown;
    return;
  }

  int newAxis = ((deltaTickCount - SYNC_PULSE_J0_MIN) / 500) & 0x1;
  if (newAxis != currentAxis) {
    //wrong sync pulse; this is for the other axis
#ifdef DEBUG_LIGHTHOUSE_EDGES
    //this technically counts as two errors, since we also mised the sweep pulse for this axes
    cycleData[currentAxis].syncMisses++;
    cycleData[currentAxis].sweepMisses++;
#endif

    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;

    //now switch to the correct axis
    currentAxis = newAxis;
  }

  captureSyncFallingEdge(previousTickCount, deltaTickCount);
}

void LighthouseSensor::captureSyncFallingEdge(unsigned int previousTickCount, unsigned int deltaTickCount)
{
  //this is the falling edge for the sync pulse along the current axis
#ifdef DEBUG_LIGHTHOUSE_EDGES
  cycleData[currentAxis].syncHits++;
  cycleData[currentAxis].syncAccumulator += (deltaTickCount - SYNC_PULSE_J0_MIN) % 500;
  cycleData[currentAxis].syncCounter++;
#endif

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

  cycleData[currentAxis].pendingCycleStart = previousTickCount;

  currentEdge = SweepRising;
}

void LighthouseSensor::processSweepRisingEdge(unsigned int previousTickCount, unsigned int currentTickCount)
{
  //this should be the rising edge of the sweep hit
  unsigned long sweepTickCount = calculateDeltaTicks(cycleData[currentAxis].pendingCycleStart, currentTickCount);
  if (sweepTickCount >= SWEEP_DURATION_LAMBDA) {
    //we must have missed the sweep hit for this axis
#ifdef DEBUG_LIGHTHOUSE_EDGES
    cycleData[currentAxis].sweepMisses++;
#endif

    //indicate that we missed the sweep on this axis
    cycleData[currentAxis].sweepHitTimeStamp = 0;

    //move to watching for the end of the sync pulse on the other axis
    currentAxis = (currentAxis+1) & 0x1;
    currentEdge = SyncFalling;
    return;
  }

  //this sweep hit is hitting within the expected sweep window; wait for the falling edge
  currentEdge = SweepFalling;
}

void LighthouseSensor::processSweepFallingEdge(unsigned int previousTickCount, unsigned int currentTickCount)
{
  //process the falling edge of the sweep hit; the sweep tick count is the number of ticks from the beginning of the sync
  //signal to the center of the sweep hit pulse
  unsigned long sweepTickCount = calculateDeltaTicks(cycleData[currentAxis].pendingCycleStart, previousTickCount) +
      (calculateDeltaTicks(previousTickCount, currentTickCount) / 2);

#ifdef DEBUG_LIGHTHOUSE_EDGES
  //collect additional sweep statistics for debugging
  cycleData[currentAxis].sweepHits++;
  cycleData[currentAxis].sweepMinTicks = cycleData[currentAxis].sweepMinTicks == 0
      ? sweepTickCount
      : min(sweepTickCount, cycleData[currentAxis].sweepMinTicks);
  cycleData[currentAxis].sweepMaxTicks = max(sweepTickCount, cycleData[currentAxis].sweepMaxTicks);
  cycleData[currentAxis].sweepAccumulator += sweepTickCount;
  cycleData[currentAxis].sweepCounter++;
#endif

  cycleData[currentAxis].sweepTickCount = sweepTickCount;
  cycleData[currentAxis].sweepHitTimeStamp = millis();

  currentEdge = SyncRising;
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
#endif

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

  //now that we have both the axis and angle of rotation, we can calculate our quaternion
  lighthouseOrientation.set(rotationUnitVector.getX(), rotationUnitVector.getY(), rotationUnitVector.getZ(), angleOfRotation);

  //take the forward unit vector in the lighthouse's coordinate system (0,1,0), and un-rotate it to get it into the global coordinate system
  KVector3 lighthouseForwardVector(0.0d, 1.0d, 0.0d);
  lighthouseForwardVector.unrotate(&lighthouseOrientation);

  //determine the height of the lighthouse from the diode plane
  double lighthouseDistanceFromDiodePlane = LIGHTHOUSE_CENTER_HEIGHT_FROM_FLOOR_MM - ROBOT_DIODE_HEIGHT_MM;

  //now we intersect the "forward" vector from the lighthouse with the diode plane to determine the relative x/y location where it's pointing
  //that location becomes our origin point in our global coordinate system; the lighthouse is considered to be offset from that location
  double t = -lighthouseDistanceFromDiodePlane / lighthouseForwardVector.getZ();
  lighthousePosition.set(-lighthouseForwardVector.getX() * t,
                         -lighthouseForwardVector.getY() * t,
                         lighthouseDistanceFromDiodePlane);

  receivedLighthousePosition = true;
}

/*
 * Translates combined x and y tick counts into a vector from the lighthouse to the zippy in the global coordinate system.
 */
void LighthouseSensor::recalculatePosition()
{
  if (!cycleData[0].sweepHitTimeStamp || !cycleData[1].sweepHitTimeStamp) {
    //we have no lighthouse signal
    return;
  }

  unsigned long newPositionTimeStamp = max(cycleData[0].sweepHitTimeStamp, cycleData[1].sweepHitTimeStamp);
  if (positionTimeStamp == newPositionTimeStamp) {
    //nothing to do; position is up-to-date
    return;
  }

  previousPositionVector.set(&positionVector);
  previousPositionTimeStamp = positionTimeStamp;

  //Step 1: Calculate the vector from the lighthouse in its reference frame to the diode.
  //start by normalizing the angle on each axis from the lighthouse to be from -90 degrees to +90 degrees in radians
  double idealAngleX = ((((double)cycleData[0].sweepTickCount) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;
  double idealAngleZ = ((((double)cycleData[1].sweepTickCount) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;

  //at y=1, we want the x and z coordinates of our direction vector; since TAN = O / A, then O = TAN / A and given that our adjacent
  //is 1.0, then the opposite is simply the TAN of the angle along each axis
  double vectorFromLighthouseX = tan(idealAngleX);
  double vectorFromLighthouseZ = tan(idealAngleZ);

  //correct for the factory calibration data; formula pulled from notes on the open source libsurvive project
  double correctedX = idealAngleX +
      xRotor.phase +
      (tan(xRotor.tilt) * vectorFromLighthouseZ) +
      (xRotor.curve * pow(vectorFromLighthouseZ, 2.0d)) +
      (sin(xRotor.gibbousPhase + idealAngleX) * xRotor.gibbousMagnitude);
  double correctedZ = idealAngleZ +
      zRotor.phase +
      (tan(zRotor.tilt) * vectorFromLighthouseX) +
      (zRotor.curve * pow(vectorFromLighthouseX, 2.0d)) +
      (sin(zRotor.gibbousPhase + idealAngleZ) * zRotor.gibbousMagnitude);

  //Step 2: Convert the vector from the lighthouse in its local coordinate system to our global coordinate system.
  //flip the x axis; it appears that our tick counts get greater from right-to-left from the perspective of the lighthouse; this is
  //contrary to some animations online which illustrate the horizontal beam sweeping from left-to-right from the perspective of the
  //lighthouse
  KVector3 directionFromLighthouse(-tan(correctedX), 1.0d, tan(correctedZ), 1.0d);
  directionFromLighthouse.unrotate(&lighthouseOrientation);

  //now intersect with the plane of the diodes on the robot; since our diode plane is defined by the normal 0,0,1, and we have a
  //vector which identifies the position of the lighthouse from 0,0,0, the entire formula for our ground-plane intersection reduces
  //to the following
  double t = -lighthousePosition.getZ() / directionFromLighthouse.getZ();
  positionVector.set(lighthousePosition.getX() + (directionFromLighthouse.getX() * t),
      lighthousePosition.getY() + (directionFromLighthouse.getY() * t));
  positionTimeStamp = newPositionTimeStamp;
}

/**
 * Estimate the current position. This is useful to cover small gaps in the detection of the lighthouse signal, but the error rate
 * will obviously grow as the time since the last detected signel increases.
 */
void LighthouseSensor::estimatePosition(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long currentTime)
{
  //calculate the change from the last known position to the position prior to that; this change occurred over the time delta between
  //those two positions, but we need to scale that over the time delta between our last known position time stamp to the new time stamp
  KVector2 deltaPosition(positionVector.getX() - previousPositionVector.getX(),
      positionVector.getY() - previousPositionVector.getY(),
      ((double)(currentTime / positionTimeStamp)) / ((double)(positionTimeStamp - previousPositionTimeStamp)));
  deltaPosition.rotate(previousOrientation->angleToVector(currentOrientation));

  previousPositionVector.set(&positionVector);
  previousPositionTimeStamp = positionTimeStamp;

  positionVector.addVector(&deltaPosition);
  positionTimeStamp = currentTime;
}

void LighthouseSensor::recalculateVelocity(KVector2* previousOrientation, KVector2* currentOrientation, unsigned long orientationTimeStamp)
{
  if (!previousPositionTimeStamp || !positionTimeStamp || velocityTimeStamp == positionTimeStamp) {
    //either we don't yet have enough position history to determine velocity or our velocity is up-to-date
    return;
  }

  KVector2 deltaPosition(positionVector.getX() - previousPositionVector.getX(), positionVector.getY() - previousPositionVector.getY());

  //poor calculation by estimating velocity as the direct distance
  //to properly calculate velocity, we need to calculate the length of the elliptical curve from the previous point to the current point
  double deltaSeconds = ((double)(positionTimeStamp - previousPositionTimeStamp)) / 1000.0d;
  velocity = deltaPosition.getD() / deltaSeconds;

  //now determine if it is negative
  if (deltaPosition.dotVector(currentOrientation) < 0.0d)
    velocity = -velocity;

  velocityTimeStamp = positionTimeStamp;
}
