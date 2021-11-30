
#ifndef _ZROUTINEOUTERCONTROLLER_
#define _ZROUTINEOUTERCONTROLLER_

#include "zippies/controllers/ZippyController.h"
#include "zippies/hardware/SensorFusor.h"

class ZRoutineOuterController : public ZippyController
{

private:
    SensorFusor* sensors;
    ZippyController* innerController;

public:
    ZRoutineOuterController(SensorFusor* s, ZippyController* innerController);
    void start(unsigned long startTime);
    void loop(unsigned long currentTime);
    void stop();

    ~ZRoutineOuterController() {}

};

#endif