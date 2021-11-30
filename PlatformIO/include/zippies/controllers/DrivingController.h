
#ifndef _DRIVINGCONTROLLER_H_
#define _DRIVINGCONTROLLER_H_

#include "zippies/controllers/ZippyController.h"
#include "zippies/ZippyMath.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/pursuit/ScissorPursuitController.h"
#include "zippies/controllers/Driver.h"

typedef enum class _DrivingState
{
    MovingIntoPlace,
    HoldingAtStart,
    SyncingWithPreamble,
    Executing,
} DrivingState;

typedef enum _DrivingMode
{
    Move,
    Watch,
    Mimic,
} DrivingMode;

/**
 * The DrivingController is intended to mimic the way that humans drive a care to a target destination
 * point with a specific target orientation. This implementation makes use of several related sub-algorithms
 * starting with the outer navigation loop in the form of the Driver class and feeding into the inner
 * control loop handled by the ScissorPursuitController class.
 */
class DrivingController : public ZippyController
{

private:
    SensorFusor* sensors;
    ScissorPursuitController pursuitController;

    ZippyWaypoint* waypoints;
    ZippyWaypointTiming* timings;
    int* commands;
    int commandCount;

    unsigned long previousTime = 0;
    unsigned long moveIntoPlaceTimer = 0;

    int currentCommand = 0;
    int currentTiming = 0;
    int currentTimingEnd = 0;
    unsigned long currentTimeRemaining = 0;
    bool isStopped = false;
    ZMatrix2 primaryAnchorPosition;
    Driver primaryDriver;
    ZMatrix2 secondaryAnchorPosition;
    Driver secondaryDriver;

    DrivingState currentState = DrivingState::MovingIntoPlace;
    DrivingMode currentDrivingMode = DrivingMode::Move;

    ZVector2 relativeShadowPosition;
    ZVector2 relativeShadowVelocity;

    void reset();
    void setCurrentMode(DrivingMode mode);

    unsigned long moveIntoPlace(unsigned long deltaTime);
    unsigned long holdAtStart(unsigned long deltaTime);
    unsigned long syncWithPreamble(unsigned long deltaTime);
    unsigned long execute(unsigned long deltaTime);

    bool processCommands();
    void initCurrentTiming();

    void pursue(unsigned long deltaTime);
    void watch(unsigned long deltaTime);
    void mimic(unsigned long deltaTime);

    void calculateRelativeTargets();

public:
    DrivingController(SensorFusor* s);
    void start(unsigned long startTime);
    void loop(unsigned long currentTime);
    void stop();

    ~DrivingController() {}

};

#endif