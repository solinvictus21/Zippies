
#ifndef _REMOTEDRIVINGCONTROLLER_H_
#define _REMOTEDRIVINGCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/Bluetooth.h"
#include "zippies/hardware/ZippyFace.h"
#include "zippies/hardware/MotorDriver.h"
#include "zippies/hardware/SensorFusor.h"

#define ENABLE_SENSORS
#define MOTOR_SEND_INTERVAL_MICROS     100000
#define SENSOR_SEND_INTERVAL_MICROS    500000

class RemoteDrivingController : public ZippyController
{

private:
    Bluetooth bluetooth;

    ZippyFace display;
    bool faceDisplayed = false;

    MotorDriver motors;
    unsigned long lastMotorSendTimestamp = 0;

#ifdef ENABLE_SENSORS
    SensorFusor sensors;
    bool sensorsReady = false;
    unsigned long lastPositionSendTimestamp = 0;
#endif

    bool started = false;

    void processBluetooth(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
#ifdef ENABLE_SENSORS
    void processSensors(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    void sendLighthouseData(bool lighthouseIsVisible);
#endif

public:
    RemoteDrivingController(const ZippyConfig* zippyConfig);

    bool start();
    bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    void stop();

    ~RemoteDrivingController() {}

};

#endif
