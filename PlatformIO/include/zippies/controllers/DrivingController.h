
#ifndef _DRIVINGCONTROLLER_H_
#define _DRIVINGCONTROLLER_H_

#ifdef WEBOTS_SUPPORT
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
using namespace webots;
#endif

#include "zippies/controllers/ZippyController.h"
#include "zippies/ZippyMath.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/pursuit/ScissorPursuitController.h"
#include "zippies/pursuit/StanleyPursuitController.h"
#include "zippies/controllers/Driver.h"

#include "zippies/config/ZippyRoutines.h"
#include "zippies/hardware/Zippy.h"

typedef enum class _DrivingState
{
    MovingIntoPlace,
    SyncingWithPreamble,
    Executing,
} DrivingState;

/**
 * The DrivingController is intended to mimic the way that humans drive a care to a target destination
 * point with a specific target orientation. This implementation makes use of several related sub-algorithms
 * starting with the outer navigation loop in the form of the Driver class and feeding into the inner
 * control loop handled by the ScissorPursuitController class.
 */
class DrivingController : public ZippyController
{

private:
#ifdef WEBOTS_SUPPORT
    //provided by the parent controller
    Field* zippyShadowTranslation;
    double translationValues[3] = { 0.0, 0.0, 0.0};
    Field* zippyShadowRotation;
    double rotationValues[4] = { 0.0, 0.0, 1.0, 0.0 };
#endif
    SensorFusor* sensors;

    //the routine to execute
    const ZippyRoutineData* routineData;

    int currentCommand = 0;
    unsigned long currentTimeRemaining = 0;
    Driver driver;
    DrivingState currentDrivingState = DrivingState::MovingIntoPlace;

    ZVector2 relativeShadowPosition;
    ZVector2 relativeShadowVelocity;

    Zippy zippy;
    double currentLinearVelocity = 0.0;
    double currentAngularVelocity = 0.0;
    bool isMoving = false;
    int stateDowngradeCounter = 0;

    void resetRoutine();

    unsigned long moveIntoPlace(unsigned long deltaTime);
    unsigned long syncWithPreamble(unsigned long deltaTime);
    unsigned long execute(unsigned long deltaTime);
    bool processCommands();

    //the pursuit controller, which translates target positions to linear/angular velocities
    ScissorPursuitController pursuitController;
    // StanleyPursuitController pursuitController; 

    void pursue(unsigned long deltaTime);
    bool completeMove();
    void executeMove();

public:
#ifdef WEBOTS_SUPPORT
    DrivingController(const ZippyConfig* zc, SensorFusor* s, Robot* zw, Node* zws);
#else
    DrivingController(const ZippyConfig* zc, SensorFusor* s);
#endif

    bool start();
    bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    void stop();

    ~DrivingController() {}

};

#endif