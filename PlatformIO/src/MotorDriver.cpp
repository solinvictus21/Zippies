
#include <Arduino.h>
#include <Wire.h>
#include "T841Defs.h"
#include "MotorDriver.h"
#include "lighthouse/Lighthouse.h"

#define MOTORS_ADDRESS 0x62
#define MOTORS_MAX_PWM_PERIOD 0xFFFF
#define MOTORS_ACCELERATION_TIME_MICROS 10000

#define COMMAND_SET_MODE 0x00  //write mode- command, register access
#define COMMAND_SERVO_1 0x01 //write 16 bit pwm value 1
#define COMMAND_SERVO_2 0x02 //write 16 bit pwm value 2
#define COMMAND_SERVO_3 0x03 //write 16 bit pwm value 3
#define COMMAND_SERVO_4 0x04 //write 16 bit pwm value 4
#define COMMAND_MOTOR_1 0x05 //write first two 16 bit pwm values
#define COMMAND_MOTOR_2 0x06 //write second two 16 bit pwm values
#define COMMAND_TIMER_1 0x08 //write 16 bit timer 1 top value
#define COMMAND_TIMER_2 0x09 //write 16 bit timer 2 top value
#define COMMAND_PRESCALER_1 0x0A //write timer 1 prescaler
#define COMMAND_PRESCALER_2 0x0B //write timer 2 prescaler
#define COMMAND_CLOCK_PRESCALER 0x0C //write system clock prescaler
#define COMMAND_SET_SLEEP_MODE 0x0D //set sleep mode
#define COMMAND_SLEEP 0x0E //go to sleep after I2C communication is done
#define COMMAND_SET_FAILSAFE_VALUES 0x0F //set failsafe PWM values - default is 0
#define COMMAND_SET_FAILSAFE_PRESCALER 0x10 //set failsafe timeout
#define COMMAND_SET_FAILSAFE_TIMEOUT 0x11 //set failsafe timeout
#define COMMAND_ALL_PWM_8 0x12 //write four 8 bit pwm values

#define T841_CLOCK_PRESCALER_1 0x00
#define T841_CLOCK_PRESCALER_2 0x01
#define T841_CLOCK_PRESCALER_4 0x02
#define T841_CLOCK_PRESCALER_8 0x03
#define T841_CLOCK_PRESCALER_16 0x04
#define T841_CLOCK_PRESCALER_32 0x05
#define T841_CLOCK_PRESCALER_64 0x06
#define T841_CLOCK_PRESCALER_128 0x07
#define T841_CLOCK_PRESCALER_256 0x08

#define T841_TIMER_PRESCALER_0 0x00
#define T841_TIMER_PRESCALER_1 0x01
#define T841_TIMER_PRESCALER_8 0x02
#define T841_TIMER_PRESCALER_64 0x03
#define T841_TIMER_PRESCALER_256 0x04
#define T841_TIMER_PRESCALER_1024 0x05

#define T841_SLEEP_MODE_IDLE 0
#define T841_SLEEP_MODE_ADC _BV(T841_SM0)
#define T841_SLEEP_MODE_PWR_DOWN _BV(T841_SM1)

#define MODE_REGISTER_INC 0xAA
#define MODE_REGISTER_DEC 0xAB
#define MODE_COMMAND 0xAC

#define FIRMWARE_REVISION_REG 0x19
#define EXPECTED_FIRMWARE 0x1A

#define _BV(bit) (1 << (bit))

MotorDriver::MotorDriver()
  : started(false)
{
}

bool MotorDriver::start()
{
  //write to the T841 registers directly
  writeByte(COMMAND_SET_MODE, MODE_REGISTER_DEC);
  uint8_t motorFirmwareVersion = read(FIRMWARE_REVISION_REG);
  if (motorFirmwareVersion == 0xFF) {
    //motors not connected
    return false;
  }
  // SerialUSB.println("Retrieved firmware version.");

  if (motorFirmwareVersion != EXPECTED_FIRMWARE) {
    //incorrect motor firmware version
    return false;
  }
  // SerialUSB.println("Got correct firmware version.");

  writeByte(T841_DDRA, _BV(7) | _BV(2) | _BV(1));
  writeByte(T841_DDRB, _BV(2));
  writeByte(T841_TOCPMSA1, _BV(T841_TOCC7S1) | _BV(T841_TOCC6S1));
  writeByte(T841_TOCPMSA0, _BV(T841_TOCC1S0) | _BV(T841_TOCC0S0));
  writeByte(T841_TCCR1A, _BV(T841_COM0A1) | _BV(T841_COM0B1) | _BV(T841_WGM11));
  writeByte(T841_TCCR2A, _BV(T841_COM2A1) | _BV(T841_COM0B1) | _BV(T841_WGM21));
  writeByte(T841_TCCR1B, _BV(T841_WGM13) | _BV(T841_WGM12) | _BV(T841_CS10));
  writeByte(T841_TCCR2B, _BV(T841_WGM23) | _BV(T841_WGM22) | _BV(T841_CS20));

  //change mode to send interpreted commands- see header file
  writeByte(COMMAND_SET_MODE, MODE_COMMAND);
  writeByte(COMMAND_CLOCK_PRESCALER, T841_CLOCK_PRESCALER_1);
  writeByte(COMMAND_PRESCALER_1, T841_TIMER_PRESCALER_1);
  writeByte(COMMAND_PRESCALER_2, T841_TIMER_PRESCALER_1);
  writeCommand(COMMAND_TIMER_1, MOTORS_MAX_PWM_PERIOD);
  writeCommand(COMMAND_TIMER_2, MOTORS_MAX_PWM_PERIOD);
  writeByte(COMMAND_SET_FAILSAFE_PRESCALER, T841_TIMER_PRESCALER_8);

  //the failsafe turns off motors if a command is not sent in a certain amount of time; we don't use it by default
  writeCommand(COMMAND_SET_FAILSAFE_TIMEOUT, 0);

  writeCommand(COMMAND_ALL_PWM, 0, 0, 0, 0);

  started = true;
  return true;
}

void MotorDriver::setFailsafe(uint16_t ms)
{
  if (ms > 0x3FFF)
    ms = 0x3FFF;

  //using defualt settings- really ~1.024ms
  writeCommand(COMMAND_SET_FAILSAFE_TIMEOUT, ms*4);
}

void MotorDriver::writeByte(uint8_t b1)
{
  Wire.beginTransmission(MOTORS_ADDRESS);
  Wire.write(b1);
  Wire.endTransmission();
}

void MotorDriver::writeByte(uint8_t b1, uint8_t b2)
{
  Wire.beginTransmission(MOTORS_ADDRESS);
  Wire.write(b1);
  Wire.write(b2);
  Wire.endTransmission();
}

void MotorDriver::writeCommand(uint8_t cmd, uint16_t val)
{
  int MSB = val >> 8;
  Wire.beginTransmission(MOTORS_ADDRESS);
  Wire.write(cmd);
  Wire.write(val);
  Wire.write(MSB);
  Wire.endTransmission();
}

void MotorDriver::writeCommand(uint8_t cmd,
  uint16_t leftForward, uint16_t leftBackward,
  uint16_t rightForward, uint16_t rightBackward)
{
  direction = ((int32_t)leftForward) - ((int32_t)leftBackward) + ((int32_t)rightForward) - ((int32_t)rightBackward);
  int MSB = leftForward >> 8;
  Wire.beginTransmission(MOTORS_ADDRESS);
  Wire.write(cmd);
  Wire.write(leftForward);
  Wire.write(MSB);
  MSB = leftBackward >> 8;
  Wire.write(leftBackward);
  Wire.write(MSB);
  MSB = rightForward >> 8;
  Wire.write(rightForward);
  Wire.write(MSB);
  MSB = rightBackward >> 8;
  Wire.write(rightBackward);
  Wire.write(MSB);
  Wire.endTransmission();
}

uint8_t MotorDriver::read(uint8_t reg)
{
  writeByte(reg);
  Wire.requestFrom(MOTORS_ADDRESS, (uint8_t)1);
  return Wire.read();
}
