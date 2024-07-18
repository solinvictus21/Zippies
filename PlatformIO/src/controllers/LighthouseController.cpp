
#include "zippies/ZippyControllers.h"

#ifdef WEBOTS_SUPPORT

LighthouseController::LighthouseController(const ZippyConfig* zippyConfig, Supervisor* zippyWebots)
  : sensors(zippyWebots)
{
//    Node* nextZippy = zippyWebots->getFromDef("RedZippy");
    Node* nextZippyShadow = zippyWebots->getFromDef("ZippyShadow");
    subController = new DrivingController(zippyConfig, &sensors, zippyWebots, nextZippyShadow);
}
#else
LighthouseController::LighthouseController(const ZippyConfig* zippyConfig)
  : sensors()
{
    subController = new DrivingController(zippyConfig, &sensors);
    // subController = new MotorTuningController(zippyConfig, &sensors);
}
#endif

bool LighthouseController::start()
{
    sensorsReady = false;
    previousPositionTimeStamp = 0;
    subControllerStartTime = 0;
    sensors.start();
    return true;
}

bool LighthouseController::loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    //wait until we have a stable signal
    if (!sensors.loop(currentTimeMicros, deltaTimeMicros)) {
        if (sensorsReady) {
            //lost the lighthouse signal
            sensorsReady = false;
            previousPositionTimeStamp = 0;
            subController->stop();
        }
        return false;
    }
    else if (!sensorsReady) {
        //acquired the lighthouse signal
        //start the sub-controller, but don't loop it until the next update so that the first delta time is not zero
        sensorsReady = true;
        // subController->start(currentTime);
        subControllerStartTime = currentTimeMicros;
        subController->start();
        return false;
    }

    //check if we have a position update
    unsigned long currentPositionTimeStamp = sensors.getPositionTimeStamp();
    if (currentPositionTimeStamp <= previousPositionTimeStamp)
        return false;
    previousPositionTimeStamp = currentPositionTimeStamp;

    subController->loop(currentTimeMicros - subControllerStartTime, deltaTimeMicros);
    return true;
}

void LighthouseController::stop()
{
    if (sensorsReady) {
        sensorsReady = false;
        previousPositionTimeStamp = 0;
        subController->stop();
    }
    sensors.stop();
}

LighthouseController::~LighthouseController()
{
    if (subController)
        delete subController;
}
