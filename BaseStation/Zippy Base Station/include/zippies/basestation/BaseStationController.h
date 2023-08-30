
#ifndef _BASESTATIONCONTROLLER_H_
#define _BASESTATIONCONTROLLER_H_

#include "zippies/hardware/Bluetooth.h"

class BaseStationController
{

private:
  Bluetooth bluetooth;

  void processBluetoothInput();
  
public:
  BaseStationController();

  void start();
  void loop();

};

#endif
