
#ifndef _BASESTATIONCONTROLLER_H_
#define _BASESTATIONCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/Bluetooth.h"

class BaseStationController : public ZippyController
{

private:
  Bluetooth bluetooth;

  void processBluetoothInput();

public:
  BaseStationController();

  void start(unsigned long currentTime);
  bool loop(unsigned long currentTime);
  void stop();

  ~BaseStationController() {}

};

#endif
