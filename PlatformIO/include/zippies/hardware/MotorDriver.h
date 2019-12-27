
#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#include <Arduino.h>

class MotorDriver
{

private:
  bool started;
  // int32_t direction = 0;
  double deadZoneRamp;
  double deadZoneRange;

  void start();
  void writeByte(uint8_t);
  void writeByte(uint8_t, uint8_t);
  void writeCommand(uint8_t, uint16_t);
  void writeCommand(uint8_t, uint16_t, uint16_t, uint16_t, uint16_t);
  uint8_t read(uint8_t);
  double rampThroughDeadZone(double a);

public:
  MotorDriver(double deadZone);
  void setFailsafe(uint16_t ms);

  void setMotors(double left, double right);
  void setMotorsDirect(double left, double right);
  void stopMotors();
  // bool inReverse() { return direction < 0; }

};

#endif
