
#include "zippies/controllers/RemoteDrivingController.h"

#define BLE_RECEIVE_MOTORS_SET 0x15
#define BLE_RECEIVE_MOTORS_ALL_STOP 0x00

RemoteDrivingController::RemoteDrivingController(const ZippyConfig* zippyConfig)
    : motors(zippyConfig)
{
}

bool RemoteDrivingController::start()
{
#ifdef ENABLE_SENSORS
    lastPositionSendTimestamp = 0;
    sensorsReady = false;
    sensors.start();
#endif
    if (!bluetooth.start())
    {
        // SerialUSB.println("Unable to start bluetooth");
        display.displayTextCentered("BT Error");
        return false;
    }

    lastMotorSendTimestamp = 0;
    display.displayTextCentered("Waiting");
    started = true;
    return false;
}

bool RemoteDrivingController::loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    if (!started)
        return false;

    processBluetooth(currentTimeMicros, deltaTimeMicros);
#ifdef ENABLE_SENSORS
    processSensors(currentTimeMicros, deltaTimeMicros);
#endif
    return true;
}

void RemoteDrivingController::processBluetooth(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    if (!bluetooth.loop(currentTimeMicros, deltaTimeMicros)) {
        //the Bluetooth layer got disrupted; this only happens when an error occurs
        if (faceDisplayed) {
            stop();
            display.clearScreen();
            const char* errorReason = bluetooth.getError();
            if (errorReason == NULL)
                errorReason = "Error";
            display.displayTextCentered(errorReason);
            faceDisplayed = false;
        }
        return;
    }

    if (!bluetooth.isClientConnected()) {
        if (faceDisplayed) {
            // SerialUSB.println("Lost connection.");
            motors.stopMotors();
            display.clearScreen();
            display.displayTextCentered("Waiting");
            faceDisplayed = false;
        }
        return;
    }
    else if (!faceDisplayed) {
        // SerialUSB.println("Connection established.");
        display.displayFace();
        faceDisplayed = true;
    }

    // process the last motor control command
    unsigned long newMotorControlTimestamp = bluetooth.getMotorControlTimestamp();

    //limit the frequency we send motor control changes to the motors
    //TODO: Always send motor stop immediately
    if (newMotorControlTimestamp <= lastMotorSendTimestamp || currentTimeMicros - lastMotorSendTimestamp < MOTOR_SEND_INTERVAL_MICROS)
        return;
    lastMotorSendTimestamp = newMotorControlTimestamp;

    // now process the requests to set the motors
    uint8_t* motorControlData = bluetooth.getMotorControlData();
    float motorLeft;
    float motorRight;
    memcpy(&motorLeft, motorControlData, 4);
    memcpy(&motorRight, motorControlData + 4, 4);

    /* log the motor control data
    SerialUSB.print("Setting motors: ");
    SerialUSB.print(motorLeft);
    SerialUSB.print(", ");
    SerialUSB.println(motorRight);
    // */

    motors.setMotorsDirect(
    // motors.setMotors(
        constrain(motorLeft, -1.0, 1.0) * 65535,
        constrain(motorRight,-1.0, 1.0) * 65535);

}

#ifdef ENABLE_SENSORS
void RemoteDrivingController::processSensors(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    if (!sensors.loop(currentTimeMicros, deltaTimeMicros)) {
        if (sensorsReady) {
            //lost the lighthouse signal
            sensorsReady = false;
            lastPositionSendTimestamp = 0;
            //send the most recent position and an indicator that the Lighthouse is no longer visible
            sendLighthouseData(false);
        }
        return;
    }
    else if (!sensorsReady) {
        //acquired the lighthouse signal
        // SerialUSB.println("Established lighthouse signal lock.");
        sensorsReady = true;
        return;
    }

    //limit the frequency that we send position updates to the client
    unsigned long newPositionTimestamp = sensors.getPositionTimeStamp();
    if (newPositionTimestamp <= lastPositionSendTimestamp || currentTimeMicros - lastPositionSendTimestamp < SENSOR_SEND_INTERVAL_MICROS)
        return;

    sendLighthouseData(true);
    lastPositionSendTimestamp = newPositionTimestamp;
}

void RemoteDrivingController::sendLighthouseData(bool lighthouseIsVisible)
{
    const ZMatrix2* currentPosition = sensors.getPosition();
    const ZMatrix2* currentDelta = sensors.getPositionDelta();
    bluetooth.sendLighthouseData(
            lighthouseIsVisible,
            currentPosition->position.getX(), currentPosition->position.getY(), currentPosition->orientation.get(),
            currentDelta->position.getD(), currentDelta->orientation.get());
}
#endif

void RemoteDrivingController::stop()
{
    motors.stopMotors();
    bluetooth.stop();
    motors.stopMotors();
#ifdef ENABLE_SENSORS
    sensors.stop();
#endif
    started = false;
}
