
#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#define COMMAND_ALL_PWM 0x07 //write four 16 bit pwm values

class MotorDriver
{

private:
  bool started;

  void writeByte(uint8_t);
  void writeByte(uint8_t, uint8_t);
  void writeCommand(uint8_t, uint16_t);
  uint8_t read(uint8_t);

public:
  MotorDriver();
  bool start();

  void setFailsafe(uint16_t ms);
  void writeCommand(uint8_t, uint16_t, uint16_t, uint16_t, uint16_t);

};

#endif
