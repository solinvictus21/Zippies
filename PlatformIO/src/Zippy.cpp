
#include "Zippy.h"

Zippy::Zippy()
{
}

//start all of our peripherals
void Zippy::start()
{
  //start the lighthouse
  lighthouse.start();

  // motors.start();
#ifdef PLATFORM_TINYSCREEN
  face.start();
  // SerialUSB.println("Started face.");

  motors.start();
  // SerialUSB.println("Started motors.");
#endif
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft < 0 ? -motorLeft: 0,
      motorLeft > 0 ? motorLeft: 0,
      motorRight < 0 ? -motorRight: 0,
      motorRight > 0 ? motorRight: 0);
}

void Zippy::loop()
{
  //first process the Lighthouse input
  lighthouse.loop();
}
