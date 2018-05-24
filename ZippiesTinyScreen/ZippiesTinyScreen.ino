
#include <SPI.h>
#include <Wire.h>
#include <STBLE.h>
#include <TinyScreen.h>
#include "MotorDriver.h"
#include "ZippyFace.h"
#include "Bluetooth.h"
#include "ZippyModes.h"
#include "Lighthouse.h"
#include "KVector.h"

#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
#define BLE_SEND_INTERVAL_MS          500

ZippyFace face;
Lighthouse lighthouse;
Bluetooth bluetooth;
unsigned long bluetoothSendDebugInfoTmeStamp = 0;
MotorDriver motors;
ZippyMode* currentMode = NULL;

bool userDriveModeEnabled = false;
  
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

void setup()
{
  Wire.begin();
  SerialUSB.begin(115200);

  //start all of our peripherals
  bluetooth.start();
  lighthouse.start();
  motors.start();
  face.start();

  //just in case the motors were left running during the last reset
  motors.setMotors(0.0d, 0.0d);
}

void loop()
{
//  /*
  //first process the Lighthouse input
  lighthouse.loop();

  processBluetoothInput();
  
  static bool lighthouseWasConnected = false;
  //now process our current drive mode; watch for "do nothing" mode
  if (currentMode != NULL)  
    currentMode->loop();
  else if (!lighthouseWasConnected && lighthouse.hasLighthouseSignal()) {
//    SerialUSB.println("Lighthouse connected. Starting auto-drive mode.");
    currentMode = new AutoDriveMode();
  }

  //now process the motors and the face
  motors.loop();
  face.loop();

  processBluetoothOutput();
}

void processBluetoothInput()
{
  //now process all the inbound Bluetooth commands
  uint8_t receivedDataLength = bluetooth.loop();
  while (receivedDataLength) {
    uint8_t* receivedData = bluetooth.getReceivedData();
    for (int i = 0; i < receivedDataLength; i++) {
      switch (receivedData[i]) {
        case BLE_AUTODRIVE_MODE:
          if (userDriveModeEnabled) {
            if (currentMode != NULL)
              delete currentMode;
            motors.setMotors(0, 0);
            currentMode = new AutoDriveMode();
            userDriveModeEnabled = false;
          }
          break;
          
        case BLE_MANUAL_MODE:
          if (!userDriveModeEnabled) {
            if (currentMode != NULL) {
              delete currentMode;
              currentMode = NULL;
            }
            motors.setMotors(0, 0);
            userDriveModeEnabled = true;
          }
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
            
            motors.setMotors(motorLeft, motorRight);
          }
          break;
          
        case BLE_RECEIVE_MOTORS_ALL_STOP:
          motors.setMotors(0, 0);
          break;
      }
    }
  
    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
  
}

void processBluetoothOutput()
{
  
  static bool bluetoothWasConnected = false;
  static bool lighthouseDataSent = false;

  bool bluetoothIsConnected = bluetooth.isConnected();
  if (bluetoothWasConnected && !bluetoothIsConnected) {
    lighthouseDataSent = false;
    if (userDriveModeEnabled) {
      //user was directly controlling the Zippy when bluetooth disconnected, to stop the motors
      userDriveModeEnabled = false;
      motors.setMotors(0, 0);
    }
  }
//  else if (!bluetoothWasConnected && bluetoothIsConnected) {
//    if (lighthouse.
//  }
  bluetoothWasConnected = bluetoothIsConnected;

//  /*
  //check to see if we need to send debug info over Bluetooth
  static uint8_t testValue = 0;
  unsigned long currentTime = millis();
  if (bluetooth.isConnected() && currentTime-bluetoothSendDebugInfoTmeStamp > BLE_SEND_INTERVAL_MS) {
//    SerialUSB.println("Sending bluetooth data.");
//    lighthouse.recalculate();
    
    float deltaTimeSeconds = ((float)(currentTime - bluetoothSendDebugInfoTmeStamp)) / 1000.0f;
    //send the sync tick count, sweep tick count, X and Y of each diode sensor
    uint8_t debugPacket[SENSOR_DATA_LENGTH];

    //left sensor data
    LighthouseSensor* sensorLeft = lighthouse.getLeftSensor();
    extractSensorPacket(sensorLeft, debugPacket);
    bluetooth.sendSensorLeft(debugPacket);

    //right sensor data
    LighthouseSensor* sensorRight = lighthouse.getRightSensor();
    extractSensorPacket(sensorRight, debugPacket);
    bluetooth.sendSensorRight(debugPacket);

    //computed data
    static float previousOrientation = 0.0f;

    KVector2* currentPosition = lighthouse.getPosition();
    
    float floatValue = currentPosition->getX();
    memcpy(debugPacket, &floatValue, sizeof(float));
    floatValue = currentPosition->getY();
    memcpy(debugPacket+4, &floatValue, sizeof(float));
    floatValue = (sensorLeft->getVelocity() + sensorRight->getVelocity()) / 2.0f;
    memcpy(debugPacket+8, &floatValue, sizeof(float));

    float orientation = lighthouse.getOrientation()->getOrientation();
    memcpy(debugPacket+12, &orientation, sizeof(float));
    float rotationalVelocityRadians = (orientation - previousOrientation) / deltaTimeSeconds;
    memcpy(debugPacket+16, &rotationalVelocityRadians, sizeof(float));
    previousOrientation = orientation;
    bluetooth.sendComputedData(debugPacket);

    bluetoothSendDebugInfoTmeStamp = currentTime;
  }
//  */
}

