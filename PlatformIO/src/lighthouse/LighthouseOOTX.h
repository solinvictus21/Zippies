
#pragma once

/*
 * Lighthouse factory calibration data for the lighthouse being used for beta testing and development.
 *
 * X Rotor Factory Calibration:
 *   Phase (radians):              1.116435
 *   Tilt (radians):               0.312331
 *   Curve (radians):             -0.070105
 *   Gibbous Phase (radians):      1.673828
 *   Gibbous Magnitude (0.0-1.0):  0.025238
 *
 * Y Rotor Factory Calibration:
 *   Phase (radians):              0.568272
 *   Tilt (radians):              -0.130265
 *   Curve (radians):              0.133325
 *   Gibbous Phase (radians):      0.238892
 *   Gibbous Magnitude (0.0-1.0): -0.007553
 */
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
