
#include "zippies/controllers/BaseStationController.h"

#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
#define BLE_SEND_INTERVAL_MS          500

#define BLE_RECEIVE_ROUTINE_DEFINITION  0x40
#define BLE_RECEIVE_START_ROUTINE       0x41
#define BLE_RECEIVE_STOP_ROUTINE        0x42

BaseStationController::BaseStationController()
{
}

void BaseStationController::start(unsigned long currentTime)
{
  if (!bluetooth.start(false)) {
    SerialUSB.println("Unable to start bluetooth");
    return;
  }
}

void BaseStationController::loop(unsigned long currentTime)
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
        case BLE_AUTODRIVE_MODE:
          // if (autoDriver == NULL)
            // autoDriver = new AutoDriveMode(&zippy);
          // autoDriver->start()
          // zippy.stop();
          break;

        case BLE_MANUAL_MODE:
          // if (autoDriver != NULL) {
            // delete autoDriver;
            // autoDriver = NULL;
          // }
          // zippy.stop();
          break;

        case BLE_RECEIVE_FORWARD_STRAIGHT:
          break;

        case BLE_RECEIVE_MOTORS_SET:
          //this command has a payload that should be 8 bytes (two signed floats)
          if (receivedDataLength-i >= 8) {
            i++;
            float motorLeft;
            memcpy(&motorLeft, receivedData+i, 4);
            i += 4;
            float motorRight;
            memcpy(&motorRight, receivedData+i, 4);
            i += 3;

            // zippy.setMotors(motorLeft, motorRight);
          }
          break;

        case BLE_RECEIVE_MOTORS_ALL_STOP:
          // zippy.stop();
          break;
      }
    }

    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
}

void BaseStationController::stop()
{

}
