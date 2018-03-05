
#pragma once

#include "LighthouseSensor.h"

class MotorDriver
{

private:
  bool started;

  void writeByte(uint8_t);
  void writeByte(uint8_t, uint8_t);
  void writeCommand(uint8_t, uint16_t);
  void writeCommand(uint8_t, uint16_t, uint16_t, uint16_t, uint16_t);
  uint8_t read(uint8_t);

public:
  MotorDriver();
  bool start();
  void setFailsafe(uint16_t ms);
  void setMotors(int32_t motorLeft, int32_t motorRight);
  void loop();
  
};

