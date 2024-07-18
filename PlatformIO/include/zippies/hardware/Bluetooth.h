
#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <STBLE.h>
#include "zippies/controllers/ZippyController.h"

#define CHAR_MOTOR_CONTROL_DATA_LENGTH 8
#define CHAR_POSITION_DATA_LENGTH 11

class Bluetooth : ZippyController
{

private:
    uint16_t gapServiceHandle;
    uint16_t deviceNameCharacteristicHandle;
    uint16_t appearanceCharacteristicHandle;

    uint16_t motorControlServiceHandle;
    uint16_t motorControlCharacteristicHandle;
    uint8_t motorControlData[CHAR_MOTOR_CONTROL_DATA_LENGTH];
    unsigned long motorControlTimestamp;

    uint16_t positionCharacteristicHandle;

    unsigned long currentTimeMicros = 0;
    bool running;

    uint16_t connectionHandle;

    const char* errorReason = NULL;

    bool startAsPeripheral();
    tBleStatus enableDiscovery();
    void packetReceived(uint8_t dataLength, uint8_t *data);

    friend void HCI_Event_CB(void *pckt);

public:
    Bluetooth();

    bool start();
    bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    bool isClientConnected() { return connectionHandle != 0; }
    uint8_t* getMotorControlData() { return motorControlData; }
    unsigned long getMotorControlTimestamp() { return motorControlTimestamp; }
    bool sendLighthouseData(bool lighthouseIsVisible, float x, float y, float orientation, float linearVelocity, float angularVelocity);
    const char* getError() { return errorReason; }
    void stop();

};

#endif
