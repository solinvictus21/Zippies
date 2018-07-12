
#include <Arduino.h>
#include "Pause.h"

Pause::Pause(Zippy* z, double seconds)
  : zippy(z),
    deltaTimeMS(seconds * 1000.0d),
    startTimeMS(0)
{
}

void Pause::start(unsigned long currentTime)
{
  zippy->stop();
  startTimeMS = currentTime;
}

bool Pause::loop(unsigned long currentTime)
{
  return (currentTime - startTimeMS) >= deltaTimeMS;
}
