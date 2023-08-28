
#include <Arduino.h>
#include <Wire.h>
#include "zippies/hardware/T841Defs.h"
#include "zippies/hardware/MotorDriver.h"
#include "zippies/hardware/MotorDriverConstants.h"
#include "zippies/config/ZippyConfig.h"

#define _BV(bit) (1 << (bit))

//the range of outputs from the PID over which to ramp through the motor dead zones
#define DEAD_ZONE_RANGE 10.0

MotorDriver::MotorDriver()
  : started(false),
    // leftDeadZoneRamp(MOTOR_DEAD_ZONE_LEFT),
    // rightDeadZoneRamp(MOTOR_DEAD_ZONE_RIGHT),
    leftWeight(1.0 - MOTOR_BALANCE),
    rightWeight(1.0 + MOTOR_BALANCE)
    // leftWeight(1.0),
    // rightWeight(1.0)
{
  // SerialUSB.println(MOTOR_DEAD_ZONE_LEFT, 1);
  // SerialUSB.println(MOTOR_DEAD_ZONE_RIGHT, 1);
  start();
}

void MotorDriver::start()
{
  //write to the T841 registers directly
  writeByte(COMMAND_SET_MODE, MODE_REGISTER_DEC);
  uint8_t motorFirmwareVersion = read(FIRMWARE_REVISION_REG);
  if (motorFirmwareVersion == 0xFF) {
    //motors not connected
    return;
  }
  // SerialUSB.println("Retrieved firmware version.");

  if (motorFirmwareVersion != EXPECTED_FIRMWARE) {
    //incorrect motor firmware version
    return;
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
  writeByte(T841_PUEB, _BV(1) | _BV(0));

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
}

void MotorDriver::setMotors(double left, double right)
{
  // left = leftWeight * rampThroughDeadZone(left, MOTOR_DEAD_ZONE);
  // right = rightWeight * rampThroughDeadZone(right, MOTOR_DEAD_ZONE);
  left = rampThroughDeadZone(leftWeight * left, MOTOR_DEAD_ZONE);
  right = rampThroughDeadZone(rightWeight * right, MOTOR_DEAD_ZONE);

  writeCommand(COMMAND_ALL_PWM,
      left > 0 ? left : 0,
      left < 0 ? -left : 0,
      right > 0 ? right : 0,
      right < 0 ? -right : 0);
      // left < 0 ? -left : 0,
      // left > 0 ? left : 0,
      // right < 0 ? -right : 0,
      // right > 0 ? right : 0);
}

void MotorDriver::setMotorsDirect(double left, double right)
{
  writeCommand(COMMAND_ALL_PWM,
      left > 0 ? left : 0,
      left < 0 ? -left : 0,
      right > 0 ? right : 0,
      right < 0 ? -right : 0);
}

void MotorDriver::stopMotors()
{
  writeCommand(COMMAND_ALL_PWM, 0, 0, 0, 0);
}

double MotorDriver::rampThroughDeadZone(double a, double deadZoneRamp)
{
  //bail out early when power output is zero
  if (a == 0.0)
    return 0.0;

  double absA = abs(a);
  /*
  double rampedValue = MOTOR_DEAD_ZONE + absA;
  if (absA < MOTOR_RAMP_ZONE) {
    double t = absA / MOTOR_RAMP_ZONE;
  */
  double rampedValue = deadZoneRamp + absA;
  if (absA < DEAD_ZONE_RANGE) {
    double t = absA / DEAD_ZONE_RANGE;
    //quadratic ramp
    // rampedValue *= (-t * (t - 2.0d));
    // rampedValue *= t * (2.0d - t);
    //cubic ramp
    rampedValue *= 1.0 + pow(t, 3.0);
  }

  return a < 0.0 ? -rampedValue : rampedValue;
  // */
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
  // direction = ((int32_t)leftForward) - ((int32_t)leftBackward) + ((int32_t)rightForward) - ((int32_t)rightBackward);
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
