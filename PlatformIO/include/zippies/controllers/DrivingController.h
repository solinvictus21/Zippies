
#ifndef _DRIVINGCONTROLLER_H_
#define _DRIVINGCONTROLLER_H_

#include "zippies/controllers/ZippyController.h"
#include "zippies/ZippyMath.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/pursuit/ScissorPursuitController.h"
#include "zippies/pursuit/StanleyPursuitController.h"
#include "zippies/controllers/Driver.h"

#include "zippies/config/ZippyRoutines.h"
#include "zippies/hardware/Zippy.h"

typedef enum class _DrivingState
{
    MovingIntoPlace,
    HoldingAtStart,
    SyncingWithPreamble,
    Executing,
} DrivingState;

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
} MovementState;

/**
 * The DrivingController is intended to mimic the way that humans drive a care to a target destination
 * point with a specific target orientation. This implementation makes use of several related sub-algorithms
 * starting with the outer navigation loop in the form of the Driver class and feeding into the inner
 * control loop handled by the ScissorPursuitController class.
 */
class DrivingController : public ZippyController
{

private:
    //on loan from parent
    SensorFusor* sensors;

    //the routine to execute
    const ZippyRoutineData* routineData;

    unsigned long moveIntoPlaceTimer = 0;
    int currentCommand = 0;
    unsigned long currentTimeRemaining = 0;
    Driver driver;
    DrivingState currentDrivingState = DrivingState::MovingIntoPlace;

    ZVector2 relativeShadowPosition;
    ZVector2 relativeShadowVelocity;

    //the pursuit controller, which translates target positions to linear/angular velocities
    // ScissorPursuitController pursuitController;
    StanleyPursuitController pursuitController;

    Zippy zippy;
    double currentLinearVelocity = 0.0;
    double currentAngularVelocity = 0.0;
    MovementState currentMovementState = MovementState::Stopped;
    int stateDowngradeCounter = 0;

    void resetRoutine();

    unsigned long moveIntoPlace(unsigned long deltaTime);
    unsigned long holdAtStart(unsigned long deltaTime);
    unsigned long syncWithPreamble(unsigned long deltaTime);
    unsigned long execute(unsigned long deltaTime);

    bool processCommands();
    void initCurrentTiming();

    void pursue(unsigned long deltaTime);
    void continuePursuit();
    void completePursuit();
    bool completeMove();
    bool completeTurn();
    void executeMove();
    

public:
    DrivingController(SensorFusor* s);
    void start();
    void loop(unsigned long deltaTime);
    void stop();

    ~DrivingController() {}

};

#endif