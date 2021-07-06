
#include "zippies/hardware/LighthouseOOTX.h"

// #define DEBUG_OOTX        1
// #define DEBUG_OOTX_ERRORS 1

#define BASE_STATION_INFO_BLOCK_SIZE 33

LighthouseOOTX::LighthouseOOTX()
{

}

void LighthouseOOTX::restart()
{
    /*
#ifdef DEBUG_OOTX_ERRORS
    SerialUSB.println("OOTX parsing restarted.");
#endif
    // */
    zeroCount = 0;
    currentParsingState = LighthouseOOTXParsingState::SyncingWithPreamble;
}

uint16_t swapBytes(uint16_t bytes)
{
    return ((bytes & 0x00FF) << 8) | ((bytes & 0xFF00) >> 8);
}

void LighthouseOOTX::processOOTXBit(unsigned int syncDelta)
{
    if (baseStationInfoBlockReceived)
        return;

    // SerialUSB.print(syncDelta);
    // SerialUSB.print("* ");
    bool bitValue = SYNC_PULSE_BIT(syncDelta);
    if (currentParsingState > LighthouseOOTXParsingState::SyncingWithPreamble) {
        syncBitCounter++;
        if (syncBitCounter == 17) {
            if (!bitValue) {
#ifdef DEBUG_OOTX_ERRORS
                SerialUSB.print("WARNING: Missed a sync bit: ");
                SerialUSB.print(readInfoBlockIndex);
                SerialUSB.print(" - ");
                SerialUSB.println(syncDelta);
#endif
                zeroCount = 0;
                currentParsingState = LighthouseOOTXParsingState::SyncingWithPreamble;
            }
            syncBitCounter = 0;
            return;
        }
    }
#ifdef DEBUG_OOTX
    else {
        SerialUSB.print("OOTX Bit: ");
        SerialUSB.print(syncDelta);
        SerialUSB.print(" - ");
        SerialUSB.println(bitValue ? "1" : "0");
    }
#endif

    switch (currentParsingState) {

        case LighthouseOOTXParsingState::SyncingWithPreamble:
            if (syncWithPreamble(bitValue)) {
#ifdef DEBUG_OOTX
                SerialUSB.println("Found the start of the OOTX frame.");
#endif
                syncBitCounter = 16;
                payloadLength = 0;
                payloadReadMask = 0x8000;
                currentParsingState = LighthouseOOTXParsingState::AcquiringOOTXPacketSize;
            }
            break;

        case LighthouseOOTXParsingState::AcquiringOOTXPacketSize:
            if (acquirePacketSize(bitValue)) {
                payloadLength = swapBytes(payloadLength);
                if (payloadLength != BASE_STATION_INFO_BLOCK_SIZE) {
#ifdef DEBUG_OOTX_ERRORS
                    SerialUSB.print("WARNING: Received an OOTX header for a packet that is NOT the base station info block of size: ");
                    SerialUSB.print(payloadLength);
                    SerialUSB.print(" - ");
                    SerialUSB.println(payloadLength, BIN);
#endif
                    restart();
              }
              else {
#ifdef DEBUG_OOTX
                  SerialUSB.println("Starting to read base station info block.");
#endif
                  readInfoBlockMask = 0x80;
                  readInfoBlockIndex = 0;
                  currentParsingState = LighthouseOOTXParsingState::ReceivingBaseStationInfoBlock;
              }
          }
          break;

        case LighthouseOOTXParsingState::ReceivingBaseStationInfoBlock:
            if (acquireInfoBlock(bitValue)) {
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

                baseStationInfoBlockReceived = true;
            }
            break;
    }
}

bool LighthouseOOTX::syncWithPreamble(bool bitValue)
{
    if (bitValue) {
        // SerialUSB.print("Preamble Reset at ");
        // SerialUSB.println(zeroCount);
        zeroCount = 0;
        return false;
    }

    zeroCount++;
    /*
    if (zeroCount > 9) {
        SerialUSB.print("Preamble Count: ");
        SerialUSB.println(zeroCount);
    }
    */
    return zeroCount == 17;
}

bool LighthouseOOTX::acquirePacketSize(bool bitValue)
{
    if (bitValue)
        payloadLength |= payloadReadMask;
    else
        payloadLength &= ~payloadReadMask;

    payloadReadMask >>= 1;
    return payloadReadMask == 0;
}

bool LighthouseOOTX::acquireInfoBlock(bool bitValue)
{
    if (bitValue)
        ((uint8_t*)&baseStationInfoBlock)[readInfoBlockIndex] |= readInfoBlockMask;
    else
        ((uint8_t*)&baseStationInfoBlock)[readInfoBlockIndex] &= ~readInfoBlockMask;

    readInfoBlockMask >>= 1;
    if (!readInfoBlockMask) {
        readInfoBlockIndex++;
        readInfoBlockMask = 0x80;
    }

    return readInfoBlockIndex == BASE_STATION_INFO_BLOCK_SIZE;
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
