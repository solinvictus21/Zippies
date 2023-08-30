
#include "zippies/basestation/BaseStationController.h"

BaseStationController::BaseStationController()
{

}

void BaseStationController::start()
{
  bluetooth.start(true);
}

void BaseStationController::loop()
{
  processBluetoothInput();
}

void BaseStationController::processBluetoothInput()
{
  //now process all the inbound Bluetooth commands
  uint8_t receivedDataLength = bluetooth.loop();
  while (receivedDataLength) {
    uint8_t* receivedData = bluetooth.getReceivedData();
    for (int i = 0; i < receivedDataLength; i++) {
      switch (receivedData[i]) {
      }
    }

    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
}
