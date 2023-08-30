
#ifndef _LIGHTHOUSECONTROLLER_H_
#define _LIGHTHOUSECONTROLLER_H_

#ifdef WEBOTS_SUPPORT
#include <webots/Supervisor.hpp>
using namespace webots;
#endif
#include "ZippyController.h"
#include "zippies/hardware/SensorFusor.h"

class LighthouseController : public ZippyController
{

private:
    SensorFusor sensors;
    bool sensorsReady = false;
    unsigned long previousPositionTimeStamp = 0;
    unsigned long subControllerDeltaTime = 0;
    ZippyController* subController;

public:
#ifdef WEBOTS_SUPPORT
    LighthouseController(Supervisor* zippyWebots);
#else
    LighthouseController();
#endif

    void start();
    bool loop(unsigned long deltaTime);
    void stop();

    ~LighthouseController();

};

#endif
