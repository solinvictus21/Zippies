
#include "LighthouseOOTX.h"

// #define DEBUG_OOTX        1
// #define DEBUG_OOTX_ERRPRS 1

#define BASE_STATION_INFO_BLOCK_SIZE 33

LighthouseOOTX::LighthouseOOTX()
{

}

void LighthouseOOTX::restart()
{
  //cancel any packet we were previously reading
  readInfoBlockMask = 0;
  zeroCount = 0;
  syncBitCounter = 0;
  payloadReadMask = 0;
  // preambleFound = false;
}

void LighthouseOOTX::processOOTXBit(unsigned int syncDelta)
{
  if (baseStationInfoBlockReceived)
    return;

  // bool value = syncDelta >= SYNC_PULSE_J1_MIN;
  bool value = ((syncDelta - SYNC_PULSE_MIN) / SYNC_PULSE_OOTX_BIT_WINDOW) & 0x1;
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
      // preambleFound = true;
#ifdef DEBUG_OOTX
      SerialUSB.println("Found the start of the OOTX frame.");
#endif

#ifdef DEBUG_OOTX_ERRPRS
      if (payloadReadMask || readInfoBlockMask)
        SerialUSB.println("WARNING: OOTX frame restarted unexpectedly.");
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
      SerialUSB.print("WARNING: Missed a sync bit: ");
      SerialUSB.println(syncDelta);
#endif
      restart();
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
        SerialUSB.println("Got the base station info block.");
        SerialUSB.println();
        SerialUSB.println("X Rotor Factory Calibration:");
        // SerialUSB.println((xRotor.phase / M_PI) * 180.0d, 10);
        // SerialUSB.println((xRotor.tilt / M_PI) * 180.0d, 10);
        // SerialUSB.println((xRotor.curve / M_PI) * 180.0d, 10);
        SerialUSB.println(getXRotorPhase(), 20);
        SerialUSB.println(getXRotorTilt(), 20);
        SerialUSB.println(getXRotorCurve(), 20);
        SerialUSB.println(getXRotorGibbousPhase(), 20);
        SerialUSB.println(getXRotorGibbousMagnitude(), 20);
        SerialUSB.println();

        SerialUSB.println("Z Rotor Factory Calibration:");
        // SerialUSB.println((zRotor.phase / M_PI) * 180.0d, 10);
        // SerialUSB.println((zRotor.tilt / M_PI) * 180.0d, 10);
        // SerialUSB.println((zRotor.curve / M_PI) * 180.0d, 10);
        SerialUSB.println(getZRotorPhase(), 20);
        SerialUSB.println(getZRotorTilt(), 20);
        SerialUSB.println(getZRotorCurve(), 20);
        SerialUSB.println(getZRotorGibbousPhase(), 20);
        SerialUSB.println(getZRotorGibbousMagnitude(), 20);
        SerialUSB.println();

        SerialUSB.print("Accelerometer: (");
        SerialUSB.print(getAccelDirX());
        SerialUSB.print(", ");
        SerialUSB.print(getAccelDirY());
        SerialUSB.print(", ");
        SerialUSB.print(getAccelDirZ());
        SerialUSB.println(")");
#endif

        //now calculate the position and orientation of the lighthouse
        baseStationInfoBlockReceived = true;
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
        SerialUSB.println("Starting to read base station info block.");
#endif
        readInfoBlockIndex = 0;
        readInfoBlockMask = 0x80;
      }
#ifdef DEBUG_OOTX_ERRPRS
      else {
        SerialUSB.print("WARNING: Receiving an OOTX frame that is NOT the base station info block of size: ");
        SerialUSB.print(payloadLength);
        SerialUSB.print(" - ");
        SerialUSB.println(payloadLength, BIN);
      }
#endif
    }
  }
  else if (readInfoBlockMask) {
    if (value)
      ((uint8_t*)&baseStationInfoBlock)[readInfoBlockIndex] |= readInfoBlockMask;
    else
      ((uint8_t*)&baseStationInfoBlock)[readInfoBlockIndex] &= ~readInfoBlockMask;

    readInfoBlockMask >>= 1;
    if (readInfoBlockMask == 0) {
      readInfoBlockIndex++;
      if (readInfoBlockIndex < BASE_STATION_INFO_BLOCK_SIZE)
        readInfoBlockMask = 0x80;
    }
  }
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
  /* old implementation
  val.u = (half & 0x7fff) << 13 | (half & 0x8000) << 16;
  if ((half & 0x7c00) != 0x7c00)
    return val.f * 0x1p112;
  val.u |= 0x7f800000;
  */

  //sign
	val.u = ((((uint32_t)half) & 0x8000) << 16);
	if (!(half & 0x7FFF))
    return val.f; //signed zero

	if (!(half & 0x7C00)) {
		//denormalized
		half = (half & 0x3FF) << 1; //only mantissa, advance intrinsic bit forward
		//shift until intrinsic bit of mantissa overflows into exponent
		//increment exponent each time
    uint8_t e = 0;
		while (!(half & 0x0400)) {
			half <<= 1;
			e++;
		}
		val.u |= ((uint32_t)(112-e)) << 23; //bias exponent to 127, half floats are biased 15 so only need to go 112 more.
		val.u |= ((uint32_t)(half & 0x3FF)) << 13; //insert mantissa
	}
	else if ((half & 0x7C00) == 0x7C00) {
		//for infinity, fraction is 0
		//for NaN, fraction is anything non zero
		//we could just copy in bits and not shift, but the mantissa of a NaN can have meaning
		val.u |= 0x7F800000 | ((uint32_t)(half & 0x3FF)) << 13;
	}
  else
    val.u |= ((((uint32_t)(half & 0x7FFF)) + 0x1C000u) << 13);

  return val.f;
}
