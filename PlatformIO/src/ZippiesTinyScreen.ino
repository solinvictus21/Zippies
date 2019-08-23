
#include <Wire.h>
#include "lighthouse/KMatrix2.h"
#include "lighthouse/KVector2.h"
#include "Zippy.h"
#include "Bluetooth.h"
#include "ZippyConfig.h"
#include "ZRoutineController.h"
#include "ZDirectController.h"

/*
#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
*/
#define BLE_SEND_INTERVAL_MS          300

#define LOOP_INDICATOR_INTERVAL 5000

Lighthouse* lighthouse;
Zippy* zippy;
ZController* controller;

#ifdef ENABLE_BLUETOOTH
Bluetooth bluetooth;
unsigned long bluetoothSendDebugInfoTmeStamp = 0;
#endif

void createZippy();

void setup()
{
  SerialUSB.begin(115200);
  // while (!SerialUSB);
  // SerialUSB.println("Started serial port.");

  Wire.begin();
  lighthouse = new Lighthouse();
  lighthouse->start();
  // SerialUSB.println("Constructed Lighthouse.");

  zippy = new Zippy();
  // SerialUSB.println("Constructed Zippy.");

#ifdef ENABLE_BLUETOOTH
  bluetooth.start();
  // SerialUSB.println("Bluetooth started.");
#endif

  controller = new ZRoutineController(lighthouse, zippy);
  // controller = new ZDirectController(lighthouse);
}

void loop()
{
  unsigned long currentTime = micros() / 1000;

// #ifdef ENABLE_BLUETOOTH
  // processBluetoothInput();
// #endif

  controller->loop(currentTime);

  // /*
#ifdef ENABLE_BLUETOOTH
  processBluetoothOutput(currentTime);
#endif
}

#ifdef ENABLE_BLUETOOTH
void processBluetoothInput()
{
  //now process all the inbound Bluetooth commands
  uint8_t receivedDataLength = bluetooth.loop();
  while (receivedDataLength) {
    uint8_t* receivedData = bluetooth.getReceivedData();
    for (int i = 0; i < receivedDataLength; i++) {
      switch (receivedData[i]) {
        case BLE_AUTODRIVE_MODE:
          if (autoDriver == NULL)
            autoDriver = new AutoDriveMode(&zippy);
          // autoDriver->start()
          zippy.stop();
          break;

        case BLE_MANUAL_MODE:
          if (autoDriver != NULL) {
            delete autoDriver;
            autoDriver = NULL;
          }
          zippy.stop();
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

            zippy.setMotors(motorLeft, motorRight);
          }
          break;

        case BLE_RECEIVE_MOTORS_ALL_STOP:
          zippy.stop();
          break;
      }
    }

    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
}

void extractSensorPacket(LighthouseSensor* sensor, uint8_t* debugPacket)
{
  //X sync ticks (2 bytes)
  unsigned short nextShortValue = 0;//sensor->getXSyncTickCount();
  memcpy(debugPacket, &nextShortValue, sizeof(unsigned short));
  int packetPosition = sizeof(unsigned short);

  //X sweep ticks (4 bytes)
  unsigned int nextIntValue = sensor->getXSweepTickCount();
  memcpy(debugPacket+packetPosition, &nextIntValue, sizeof(unsigned int));
  packetPosition += sizeof(unsigned int);

  //calculated X position (4 bytes)
  KVector2* currentSensorPosition = sensor->getPosition();
  float nextFloatValue = currentSensorPosition->getX();
  memcpy(debugPacket+packetPosition, &nextFloatValue, sizeof(float));
  packetPosition += sizeof(float);

  //Y sync ticks (2 bytes)
  nextShortValue = 0;//sensor->getYSyncTickCount();
  memcpy(debugPacket+packetPosition, &nextShortValue, sizeof(unsigned short));
  packetPosition += sizeof(unsigned short);

  //Y sweep ticks (4 bytes)
  nextIntValue = sensor->getYSweepTickCount();
  memcpy(debugPacket+packetPosition, &nextIntValue, sizeof(unsigned int));
  packetPosition += sizeof(unsigned int);

  //calculated Y position (4 bytes)
  nextFloatValue = currentSensorPosition->getY();
  memcpy(debugPacket+packetPosition, &nextFloatValue, sizeof(float));
}

void processBluetoothOutput(unsigned long currentTime)
{
  //check to see if we need to send debug info over Bluetooth
  if (currentTime - bluetoothSendDebugInfoTmeStamp < BLE_SEND_INTERVAL_MS)
    return;

  // SerialUSB.println("Sent updated data.");
  const KMatrix2* currentPosition = lighthouse.getPosition();
  const KMatrix2* currentPositionDelta = lighthouse.getPositionDelta();
  KVector2 relativeVelocity(&currentPositionDelta->position);
  double previousOrientation = subtractAngles(currentPosition->orientation.get(), currentPositionDelta->orientation.get());
  relativeVelocity.rotate(-previousOrientation);

  bluetooth.sendBroadcastData(
      currentPosition->orientation.get(),
      relativeVelocity.getX(),
      relativeVelocity.getY(),
      relativeVelocity.getD(),
      relativeVelocity.atan2()
    );

  bluetoothSendDebugInfoTmeStamp = currentTime;
}
#endif //ENABLE_BLUETOOTH
