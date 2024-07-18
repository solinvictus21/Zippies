
#ifndef _LIGHTHOUSECONTROLLER_H_
#define _LIGHTHOUSECONTROLLER_H_

#ifdef WEBOTS_SUPPORT
#include <webots/Supervisor.hpp>
using namespace webots;
#endif
#include "ZippyController.h"
#include "zippies/config/ZippyRoutines.h"
#include "zippies/hardware/SensorFusor.h"

class LighthouseController : public ZippyController
{

private:
    SensorFusor sensors;
    bool sensorsReady = false;
    unsigned long previousPositionTimeStamp = 0;
    unsigned long subControllerStartTime = 0;
    ZippyController* subController;

public:
#ifdef WEBOTS_SUPPORT
    LighthouseController(const ZippyConfig* zippyConfig, Supervisor* zippyWebots);
#else
    LighthouseController(const ZippyConfig* zippyConfig);
#endif

    bool start();
    bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    void stop();

    ~LighthouseController();

};

#endif
