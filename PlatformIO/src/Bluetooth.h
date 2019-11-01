
#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <STBLE.h>

// #define SENSOR_DATA_LENGTH 20
#define ADVERTISEMENT_PACKET_LENGTH 31

class Bluetooth
{

private:
  uint16_t serviceHandle;
  uint16_t serviceNameHandle;
  uint16_t apperanceNameHandle;

  uint16_t primaryServiceHandle;
  uint16_t characteristicTransmitHandle;
  uint16_t lighthouseReceiveHandle;
  // uint16_t sensorLeftReceiveHandle;
  // uint16_t sensorRightReceiveHandle;
  // uint16_t computedDataReceiveHandle;

  bool discoveryEnabled;
  bool started;

  uint16_t connectionHandle;

  uint8_t* receivedData;
  uint8_t receivedDataLength;

/*
  uint8_t broadcastData[ADVERTISEMENT_PACKET_LENGTH] = {
    3, AD_TYPE_16_BIT_SERV_UUID, 0x0B, 0xB0,
    26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0
  };
*/

  bool startAsPeripheral();
  // bool startAsBroadcaster();
  // bool startAsObserver();
  bool enableDiscovery();
  friend void HCI_Event_CB(void *pckt);
  void packetReceived(uint8_t dataLength, uint8_t *data);

public:
  Bluetooth();

  bool start();
  uint8_t loop();
  // void sendBroadcastData(float x, float y, float orientation, float linearVelocity, float targetVelocity);
  bool sendLighthouseData(float x, float y, float orientation, float linearVelocity, float angularVelocity);
  /*
  tBleStatus sendLighthouseData(uint8_t* lighthouseDataBuffer);
  tBleStatus sendSensorLeft(uint8_t* sendBuffer);
  tBleStatus sendSensorRight(uint8_t* sendBuffer);
  tBleStatus sendComputedData(uint8_t* sendBuffer);
  */
  uint8_t getReceivedDataLength() { return receivedDataLength; }
  uint8_t* getReceivedData() { return receivedData; }
  void stop();

  bool isConnected();

};

#endif
