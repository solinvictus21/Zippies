
#ifndef _LIGHTHOUSEOOTX_H_
#define _LIGHTHOUSEOOTX_H_

#include <Arduino.h>

//use the full 180 degrees to determine the actual range of ticks
#define SWEEP_DURATION_TICK_COUNT 400000
#define SYNC_PULSE_AXIS_WINDOW       500
#define SYNC_PULSE_OOTX_BIT_WINDOW  1000
#define SYNC_PULSE_MIN              2945
#define SYNC_PULSE_MAX              6945
//x axis, OOTX bit 0
#define SYNC_PULSE_J0_MIN           SYNC_PULSE_J0_MIN
//y axis, OOTX bit 0
#define SYNC_PULSE_K0_MIN           3445
//x axis, OOTX bit 1
#define SYNC_PULSE_J1_MIN           3945
//y axis, OOTX bit 1
#define SYNC_PULSE_K1_MIN           4445
//x axis, OOTX bit 0, skip
#define SYNC_PULSE_J2_MIN           4945
//y axis, OOTX bit 0, skip
#define SYNC_PULSE_K2_MIN           5445
//x axis, OOTX bit 1, skip
#define SYNC_PULSE_J3_MIN           5945
//y axis, OOTX bit 1, skip
#define SYNC_PULSE_K3_MIN           6445

#define SYNC_PULSE_NUMBER(a)        (a - SYNC_PULSE_MIN) / SYNC_PULSE_AXIS_WINDOW
#define SYNC_PULSE_AXIS(a)          (SYNC_PULSE_NUMBER(a)) & 0x1
#define SYNC_PULSE_BIT(a)           ((a - SYNC_PULSE_MIN) / SYNC_PULSE_OOTX_BIT_WINDOW) & 0x1

float float16ToFloat32(uint16_t half);

//we need the base station info block struct to be byte-aligned; otherwise it'll be aligned according to the MCU
//we're running on (32 bits for SAMD21) and the data we want from it will be unintelligible; hence these pragmas
//several of these values are actually 16-bit floating point numbers, but since our platform doesn't have those, we treat them
//as unsigned integers for the purpose of allocating space and will have to manually convert them later
#pragma pack(push)
#pragma pack(1)
typedef struct _BaseStationInfoBlock {
  uint16_t fw_version;
  uint32_t id;
  //phase is an artifact caused by the linear offset of the laser from the ideal; at a phase of zero, the laser follows the ideal
  //a negative phase indicates that the laser will trail slightly, and a positive phase indicates that the laser will lead slightly
  uint16_t fcal_0_phase;
  uint16_t fcal_1_phase;
  uint16_t fcal_0_tilt;
  uint16_t fcal_1_tilt;
  uint8_t sys_unlock_count;
  uint8_t hw_version;
  uint16_t fcal_0_curve;
  uint16_t fcal_1_curve;
  //the following three values indicate the "up" vector of the lighthouse; so for example, a perfectly upright lighthouse would have
  //an accel vector of 0, 127, 0; the front faces 0, 0, 127 from the lighthouse internal coordinate system
  //x axis is right (-) to left (+) from the perspective of the lighthouse
  int8_t accel_dir_x;
  //y axis is down (-) to up (+)
  int8_t accel_dir_y;
  //z axis is back (-) to front (+)
  int8_t accel_dir_z;

  uint16_t fcal_0_gibphase;
  uint16_t fcal_1_gibphase;

  //gibbous magnitude is an artifact caused by distance of the laser from the lens compared to the ideal; at a magnitude of zero, the
  //laser is visible starting exactly 1/6th of a second after the start of the sync flash and then sweeps through the 120-degree field
  //of view in exactly 2/3rds of a second; positive values indicate that it starts later and ends sooner; negative values indicate
  //that it starts sooner and ends later
  uint16_t fcal_0_gibmag;
  uint16_t fcal_1_gibmag;
  uint8_t mode_current;
  uint8_t sys_faults;
} BaseStationInfoBlock;
#pragma pack(pop)

typedef enum class _LighthouseOOTXParsingState
{
  SyncingWithPreamble,
  AcquiringOOTXPacketSize,
  ReceivingBaseStationInfoBlock,
} LighthouseOOTXParsingState;

class LighthouseOOTX
{

private:
  //data and code required to process the OOTX frame, for the purpose of extracting the lighthouse orientation
  LighthouseOOTXParsingState currentParsingState = LighthouseOOTXParsingState::SyncingWithPreamble;

  int zeroCount = 0;
  int syncBitCounter = 0;
  unsigned short payloadLength = 0;
  unsigned short payloadReadMask = 0;
  int readInfoBlockIndex = 0;
  byte readInfoBlockMask = 0;

  BaseStationInfoBlock baseStationInfoBlock;
  bool baseStationInfoBlockReceived = false;

  bool syncWithPreamble(bool bitValue);
  bool acquirePacketSize(bool bitValue);
  bool acquireInfoBlock(bool bitValue);

public:
  LighthouseOOTX();

  void restart();
  void processOOTXBit(unsigned int syncDelta);

  bool receivedBaseStationInfoBlock() const { return baseStationInfoBlockReceived; }

  double getXRotorPhase() const { return float16ToFloat32(baseStationInfoBlock.fcal_0_phase); }
  double getXRotorTilt() const { return float16ToFloat32(baseStationInfoBlock.fcal_0_tilt); }
  double getXRotorCurve() const { return float16ToFloat32(baseStationInfoBlock.fcal_0_curve); }
  double getXRotorGibbousPhase() const { return float16ToFloat32(baseStationInfoBlock.fcal_0_gibphase); }
  double getXRotorGibbousMagnitude() const { return float16ToFloat32(baseStationInfoBlock.fcal_0_gibmag); }

  double getZRotorPhase() const { return float16ToFloat32(baseStationInfoBlock.fcal_1_phase); }
  double getZRotorTilt() const { return float16ToFloat32(baseStationInfoBlock.fcal_1_tilt); }
  double getZRotorCurve() const { return float16ToFloat32(baseStationInfoBlock.fcal_1_curve); }
  double getZRotorGibbousPhase() const { return float16ToFloat32(baseStationInfoBlock.fcal_1_gibphase); }
  double getZRotorGibbousMagnitude() const { return float16ToFloat32(baseStationInfoBlock.fcal_1_gibmag); }

  double getAccelDirX() const { return baseStationInfoBlock.accel_dir_x; }
  double getAccelDirY() const { return baseStationInfoBlock.accel_dir_y; }
  double getAccelDirZ() const { return baseStationInfoBlock.accel_dir_z; }

};

#endif
