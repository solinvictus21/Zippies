
#pragma once
#include <STBLE.h>
#include <arduino_bluenrg_ble.h>

#define SENSOR_DATA_LENGTH 20

class Bluetooth
{
private:
  bool started;
  uint16_t serviceHandle;
  uint16_t characteristicTransmitHandle;
  uint16_t sensorRightReceiveHandle;
  uint16_t sensorLeftReceiveHandle;
  uint16_t computedDataReceiveHandle;
  bool discoveryEnabled;
  uint16_t connectionHandle;

  uint8_t* receivedData;
  uint8_t receivedDataLength;

  bool enableDiscovery();
  friend void HCI_Event_CB(void *pckt);
  void packetReceived(uint8_t dataLength, uint8_t *data);

public:
  Bluetooth();
  
  bool start();
  uint8_t loop();
  tBleStatus sendSensor0(uint8_t* sendBuffer);
  tBleStatus sendSensor1(uint8_t* sendBuffer);
  tBleStatus sendComputedData(uint8_t* sendBuffer);
  uint8_t getReceivedDataLength() { return receivedDataLength; }
  uint8_t* getReceivedData() { return receivedData; }
  void stop();

  bool isConnected();
};

