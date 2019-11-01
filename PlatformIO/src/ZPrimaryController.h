
#ifndef _ZPRIMARYCONTROLLER_H_
#define _ZPRIMARYCONTROLLER_H_

#include "ZController.h"
#include "lighthouse/SensorFusor.h"
#include "Zippy.h"
#include "ZippyConfig.h"

#ifdef BLUETOOTH_ENABLED
#include "Bluetooth.h"

/*
#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
*/
#define BLE_SEND_INTERVAL_MS          300
#endif //BLUETOOTH_ENABLED

class ZPrimaryController : public ZController
{

private:
  SensorFusor sensors;
  bool sensorsReady = false;
  unsigned long previousPositionTimeStamp = 0;

  Zippy zippy;

#ifdef BLUETOOTH_ENABLED
  Bluetooth bluetooth;

  void processBluetoothInput(unsigned long currentTime);
  void processBluetoothOutput(unsigned long currentTime);
#endif

  ZController* subController;

public:
  ZPrimaryController();

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);
  void stop();

  ~ZPrimaryController();

};

#endif
