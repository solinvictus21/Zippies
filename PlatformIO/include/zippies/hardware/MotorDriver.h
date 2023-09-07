
#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#ifdef WEBOTS_SUPPORT
#include <stdlib.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
using namespace webots;
#else
#include <Arduino.h>
#endif

#include "zippies/config/ZippyRoutines.h"

class MotorDriver
{

private:
#ifdef WEBOTS_SUPPORT
    Motor* leftWheelMotor = NULL;
    Motor* rightWheelMotor = NULL;
#endif
  double leftStictionPower;
  double leftDeadZone;
  double leftGain;
  double leftCurrentPower = 0.0;
  double rightStictionPower;
  double rightDeadZone;
  double rightGain;
  double rightCurrentPower = 0.0;

  void start();
  void writeByte(uint8_t);
  void writeByte(uint8_t, uint8_t);
  void writeCommand(uint8_t, uint16_t);
  void writeCommand(uint8_t, uint16_t, uint16_t, uint16_t, uint16_t);
  uint8_t read(uint8_t);
  double rampThroughDeadZone(double a, double deadZoneRamp);

public:
#ifdef WEBOTS_SUPPORT
    MotorDriver(const ZippyConfig* zippyConfig, Robot* zippyWebots);
#else
    MotorDriver(const ZippyConfig* zippyConfig);
#endif
  void setFailsafe(uint16_t ms);

  void setMotors(double left, double right);
  void setMotorsDirect(double left, double right);
  void stopMotors();

};

#endif
