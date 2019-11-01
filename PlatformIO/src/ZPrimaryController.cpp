
#include <Arduino.h>
#include "ZPrimaryController.h"
#include "ZRoutineController.h"
#include "ZTuningController.h"

ZPrimaryController::ZPrimaryController()
{
  // subController = new ZRoutineController(&sensors, &zippy);
  subController = new ZTuningController(&sensors, &zippy);
}

void ZPrimaryController::start(unsigned long currentTime)
{
  SerialUSB.begin(115200);
  // while (!SerialUSB);
  // SerialUSB.println("Started serial port.");

  sensorsReady = false;
  previousPositionTimeStamp = 0;
  sensors.start();

#ifdef ENABLE_BLUETOOTH
  bluetooth.start();
#endif
}

void ZPrimaryController::loop(unsigned long currentTime)
{
#ifdef ENABLE_BLUETOOTH
  processBluetoothInput(currentTime);
#endif

  //wait until we have a stable signal
  if (!sensors.loop(currentTime)) {
    if (sensorsReady) {
      // SerialUSB.println("Lighthouse signal lost.");
      sensorsReady = false;
      previousPositionTimeStamp = 0;
      subController->stop();
      zippy.stop();
    }
    return;
  }

  //check if we have a position update
  if (sensors.getPositionTimeStamp() <= previousPositionTimeStamp)
    return;
  previousPositionTimeStamp = sensors.getPositionTimeStamp();

  if (!sensorsReady) {
    // SerialUSB.println("Lighthouse signal locked.");
    sensorsReady = true;
    zippy.start();
    subController->start(currentTime);
    return;
  }

  zippy.setInputs(sensors.getPosition(), sensors.getPositionDelta());
  subController->loop(currentTime);
  zippy.loop();

#ifdef ENABLE_BLUETOOTH
  processBluetoothInput(currentTime);
#endif
}

void ZPrimaryController::stop()
{
  subController->stop();
  sensors.stop();

#ifdef ENABLE_BLUETOOTH
  bluetooth.stop();
#endif
}

#ifdef ENABLE_BLUETOOTH
void ZPrimaryController::processBluetoothInput(unsigned long currentTime)
{
  //now process all the inbound Bluetooth commands
  uint8_t receivedDataLength = bluetooth.loop();
  while (receivedDataLength) {
    uint8_t* receivedData = bluetooth.getReceivedData();
    for (int i = 0; i < receivedDataLength; i++) {
      switch (receivedData[i]) {
        /*
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
        */
      }
    }

    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
}

/*
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
  const KVector2* currentSensorPosition = sensor->getPosition();
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
*/

void ZPrimaryController::processBluetoothOutput(unsigned long currentTime)
{
  //check to see if we need to send debug info over Bluetooth
  if (currentTime - bluetoothSendDebugInfoTmeStamp < BLE_SEND_INTERVAL_MS)
    return;

  // SerialUSB.println("Sent updated data.");
  const KMatrix2* currentPosition = lighthouse->getPosition();
  const KMatrix2* currentPositionDelta = lighthouse->getPositionDelta();
  /*
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
  */

  bluetooth.sendLighthouseData(
      currentPosition->position.getX(),
      currentPosition->position.getY(),
      currentPosition->orientation.get(),
      currentPositionDelta->position.getD(),
      currentPositionDelta->orientation.get());

  bluetoothSendDebugInfoTmeStamp = currentTime;
}
#endif //ENABLE_BLUETOOTH

ZPrimaryController::~ZPrimaryController()
{
  if (subController)
    delete subController;
}
