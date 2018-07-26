
#include <Wire.h>
#include "Platform.h"
#include "Zippy.h"
#include "AutoDriveMode.h"
#include "Bluetooth.h"
#include "ZippyConfig.h"

#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
#define BLE_SEND_INTERVAL_MS          500

#define LOOP_INDICATOR_INTERVAL 5000

Zippy zippy;

#ifdef ENABLE_BLUETOOTH
Bluetooth bluetooth;
unsigned long bluetoothSendDebugInfoTmeStamp = 0;
#endif

AutoDriveMode* autoDriver = NULL;

// void processBluetoothInput();
// void processBluetoothOutput();

void setup()
{
  Wire.begin();
  SerialUSB.begin(115200);
  // while (!SerialUSB);
  // Serial.println("Started serial port.");

#ifdef ENABLE_BLUETOOTH
  bluetooth.start();
#endif

  zippy.start();
  // SerialUSB.println("Started Zippy.");

  autoDriver = new AutoDriveMode(&zippy);
  // SerialUSB.println("Started Auto-Drive mode.");

  pinMode(8, OUTPUT); //Onboard Blue LED
  digitalWrite(8, HIGH);
}

void loop()
{
  static uint64_t loopTimer = millis();

#ifdef ENABLE_BLUETOOTH
  processBluetoothInput();
#endif

  zippy.loop();

  if (autoDriver != NULL)
    autoDriver->loop();

#ifdef ENABLE_BLUETOOTH
  processBluetoothOutput();
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
// */

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

void processBluetoothOutput()
{
  //check to see if we need to send debug info over Bluetooth
  unsigned long currentTime = millis();
  if (/*!bluetooth.isConnected() || */currentTime-bluetoothSendDebugInfoTmeStamp < BLE_SEND_INTERVAL_MS)
    return;

  // SerialUSB.println("Sent updated data.");
  Lighthouse* lighthouse = zippy.getLighthouse();
  KVector2* positionVector = lighthouse->getPosition();
  KVector2* orientationVector = lighthouse->getOrientation();
  KVector2* velocityVector = lighthouse->getVelocity();
  bluetooth.sendBroadcastData(positionVector->getX(), positionVector->getY(),
      orientationVector->getOrientation(), velocityVector->getD());
  /*
  float deltaTimeSeconds = ((float)(currentTime - bluetoothSendDebugInfoTmeStamp)) / 1000.0f;

  //send the sync tick count, sweep tick count, X and Y of each diode sensor
  uint8_t debugPacket[SENSOR_DATA_LENGTH];

  //left sensor data
  LighthouseSensor* sensorLeft = lighthouse->getLeftSensor();
  extractSensorPacket(sensorLeft, debugPacket);
  bluetooth.sendSensorLeft(debugPacket);

  //right sensor data
  LighthouseSensor* sensorRight = lighthouse->getRightSensor();
  extractSensorPacket(sensorRight, debugPacket);
  bluetooth.sendSensorRight(debugPacket);

  //computed data
  static float previousOrientation = 0.0f;

  KVector2* currentPosition = lighthouse->getPosition();

  float floatValue = currentPosition->getX();
  memcpy(debugPacket, &floatValue, sizeof(float));
  floatValue = currentPosition->getY();
  memcpy(debugPacket+4, &floatValue, sizeof(float));
  floatValue = (sensorLeft->getVelocity()->getD() + sensorRight->getVelocity()->getD()) / 2.0f;
  memcpy(debugPacket+8, &floatValue, sizeof(float));

  float orientation = lighthouse->getOrientation()->getOrientation();
  memcpy(debugPacket+12, &orientation, sizeof(float));
  float rotationalVelocityRadians = (orientation - previousOrientation) / deltaTimeSeconds;
  memcpy(debugPacket+16, &rotationalVelocityRadians, sizeof(float));
  previousOrientation = orientation;
  bluetooth.sendComputedData(debugPacket);
  */

  bluetoothSendDebugInfoTmeStamp = currentTime;
}
#endif
