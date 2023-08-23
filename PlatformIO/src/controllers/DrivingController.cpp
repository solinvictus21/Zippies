
#include "zippies/controllers/DrivingController.h"
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/pursuit/PursuitController.h"
#include "zippies/ZippyMath.h"

#define MOVE_TO_START_TIMING                       5000
#define MOVE_TO_START_VELOCITY                      300.0

#define STATE_DOWNGRADE_MOVING                        5
#define STATE_DOWNGRADE_TURNING                      20

// #define MAX_POSITIONING_MOVES                       10
// #define MAX_ORIENTATION_MOVES                       20

// #define LINEAR_EPSILON                                1.00  //0.1cm
// #define LINEAR_EPSILON                                5.00  //0.5cm
#define LINEAR_EPSILON                                8.00
// #define LINEAR_EPSILON                               10.00  //1.0cm
// #define LINEAR_EPSILON                               20.00  //2cm
// #define LINEAR_EPSILON                               30.00  //3cm

#define ANGULAR_EPSILON                               0.052359877559830  //3 degrees
// #define ANGULAR_EPSILON                               0.069813170079773  //4 degrees
// #define ANGULAR_EPSILON                               0.087266462599716  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149  //15 degrees

#define LIMIT_LINEAR_ACCELERATION_MIN                 2.0
#define LIMIT_LINEAR_ACCELERATION                     0.5
#define LIMIT_LINEAR_VELOCITY_MAX                    50.0

DrivingController::DrivingController(SensorFusor* s)
  : sensors(s)
{
    routineData = getZippyRoutineData();
}

void DrivingController::start()
{
    /* TEMP: TEST THE DRIVER
    SerialUSB.println("Testing Driver");
    driver.reset();
    driver.start(
        100, 100.0, 90.0 * DEG2RAD,
        0, 0.0, 0.0 * DEG2RAD);
    for (int i = 160; i >= 0; i -= 16)
        driver.update(i);

    SerialUSB.println("HOLDING IN POSITION: ");
    driver.getShadowPosition()->printDebug();
    driver.getShadowToTargetPosition()->printDebug();
    driver.getShadowVelocity()->printDebug();

    driver.holdPosition();

    driver.getShadowPosition()->printDebug();
    driver.getShadowVelocity()->printDebug();
    // */

    //this function is only called when the Zippy loses and then reacquires the LIghthouse signal
    //in this scenario, we need to do a full restart of the driving capabilities
    resetRoutine();
    driver.reset();

    //setup the initial move to the starting position
    driver.start(sensors->getPosition(),
        routineData->startingPosition[0],
        routineData->startingPosition[1],
        routineData->startingPosition[2] * DEG2RAD);

    //setup the timings to drive to the start position
    currentTimeRemaining = min(
        (unsigned long)(1000.0 * driver.getDistanceToTarget() / MOVE_TO_START_VELOCITY),
        (unsigned long)MOVE_TO_START_TIMING);
    moveIntoPlaceTimer = MOVE_TO_START_TIMING;
    currentDrivingState = DrivingState::MovingIntoPlace;
}

void DrivingController::resetRoutine()
{
    //reset all the variables used to execute routines
    currentCommand = 0;
    currentTimeRemaining = 0;
}

void DrivingController::loop(unsigned long deltaTime)
{
    //this function is invoked by the outer loop whenever a new position has become available from the Lighthouse
    while (deltaTime) {
        switch (currentDrivingState) {
            case DrivingState::MovingIntoPlace:
                deltaTime = moveIntoPlace(deltaTime);
                break;

            case DrivingState::HoldingAtStart:
                deltaTime = holdAtStart(deltaTime);
                break;

            case DrivingState::SyncingWithPreamble:
                deltaTime = syncWithPreamble(deltaTime);
                break;

            case DrivingState::Executing:
                deltaTime = execute(deltaTime);
                break;
        }
    }
}

unsigned long DrivingController::moveIntoPlace(unsigned long deltaTime)
{
    if (deltaTime > currentTimeRemaining)// ||
        // abs(driver.getLinearVelocity()) > driver.getDistanceToTarget())
    {
        //we've reached the starting position
        deltaTime -= currentTimeRemaining;
        moveIntoPlaceTimer -= currentTimeRemaining;
        currentTimeRemaining = 0;

        //hold at the starting position until the startup timer expires
        // SerialUSB.println("Debug hold #2");
        driver.holdPosition();
        currentDrivingState = DrivingState::HoldingAtStart;

        return deltaTime;
    }

    currentTimeRemaining -= deltaTime;
    moveIntoPlaceTimer -= deltaTime;
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::holdAtStart(unsigned long deltaTime)
{
    if (deltaTime > moveIntoPlaceTimer) {
        //initial movement time expired
        deltaTime -= moveIntoPlaceTimer;
        moveIntoPlaceTimer = 0;

        //sync with the preamble before starting routine so that all robots are potentially on the same timing
        sensors->syncWithPreamble();
        currentDrivingState = DrivingState::SyncingWithPreamble;
        return deltaTime;
    }

    moveIntoPlaceTimer -= deltaTime;
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::syncWithPreamble(unsigned long deltaTime)
{
    if (sensors->foundPreamble()) {
        //begin execution of the routine
        resetRoutine();

        //setup the first command
        processCommands();

        //start executing the routine
        currentDrivingState = DrivingState::Executing;
        return deltaTime;
    }

    //continue waiting
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::execute(unsigned long deltaTime)
{
    if (deltaTime > currentTimeRemaining /* ||
        abs(driver.getLinearVelocity()) > driver.getDistanceToTarget()*/)
    {
        //check if we have more commands to process
        if (!processCommands()) {
            //sync with the preamble
            sensors->syncWithPreamble();
            currentDrivingState = DrivingState::SyncingWithPreamble;
            return deltaTime;
        }
    }
    
    currentTimeRemaining -= deltaTime;
    pursue(deltaTime);
    return 0;
}

bool DrivingController::processCommands()
{
    if (currentCommand >= routineData->routineMovementCount) {
        //we've reached the end of the routine
        // SerialUSB.println("Debug hold #3");
        driver.holdPosition();
        return false;
    }

    int timing = routineData->routineMovements[currentCommand++];

    //increment the timing
    currentTimeRemaining += abs(timing);
    if (timing < 0) {
        //we're temporarily stopping at this position
        driver.holdPosition();

        /*
        SerialUSB.print("HOLDING IN POSITION: ");
        SerialUSB.println(driver.isReverseDirection());
        driver.getTargetPosition()->printDebug();
        driver.getShadowVelocity()->printDebug();
        */

        return true;
    }

    //pull the waypoint
    driver.setTargetPosition(
        routineData->routineMovements[currentCommand],
        routineData->routineMovements[currentCommand+1],
        routineData->routineMovements[currentCommand+2] * DEG2RAD);
    currentCommand += 3;

    //just in case we were in the middle of a state downgrade, reset the downgrade counter
    stateDowngradeCounter = STATE_DOWNGRADE_MOVING;

    return true;
}

void DrivingController::pursue(unsigned long deltaTime)
{
    driver.update(max(currentTimeRemaining, deltaTime));

    zippy.setInput(sensors->getPositionDelta());

    /* helps look at the individual points coming out of the driver
    if (currentState == DrivingState::Executing) {
        SerialUSB.print("target: ");
        SerialUSB.print(micros());
        SerialUSB.print(",  ");
        driver.getShadowPosition()->printDebug();
        const ZVector2* test = driver.getShadowVelocity();
        SerialUSB.print("velocity: ");
        SerialUSB.print(test->getX());
        SerialUSB.print(",  ");
        SerialUSB.print(test->getY());
        SerialUSB.print(",  ");
        SerialUSB.print(RAD2DEG * test->atan2());
        SerialUSB.println();
        // driver.getShadowVelocity()->printDebug();
    }
    */

    //calculate vectors relative to current position
    const ZMatrix2* currentPosition = sensors->getPosition();
    relativeShadowPosition.set(driver.getShadowPosition());
    relativeShadowPosition.subtract(&currentPosition->position);
    relativeShadowPosition.unrotate(&currentPosition->orientation);
    relativeShadowVelocity.set(driver.getShadowVelocity());
    relativeShadowVelocity.unrotate(&currentPosition->orientation);

    if (!driver.isHoldingPosition())
        continuePursuit();
    else
        completePursuit();
}

void DrivingController::continuePursuit()
{
    //the "normal" behavior of just pursuit the target around the stage
    //upgrade our state to "moving"
    switch (currentMovementState) {
        case MovementState::Turning:
            //upgrade from turning to moving
            stateDowngradeCounter = STATE_DOWNGRADE_MOVING;
            currentMovementState = MovementState::Moving;
            break;

        case MovementState::Stopped:
            //start moving
            zippy.start();
            stateDowngradeCounter = STATE_DOWNGRADE_MOVING;
            currentMovementState = MovementState::Moving;
            break;
    }

    executeMove();
}

void DrivingController::completePursuit()
{
    //stop at the position
    /*
    zippy.stop();
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    currentMovementState = MovementState::Stopped;
    */

    // /*
    //continue downgrading the movement state until stopped
    switch (currentMovementState) {
        case MovementState::Moving:
            // if (!completeMove())
                // return;

            //downgrade movement states until stopped at the position
            stateDowngradeCounter = STATE_DOWNGRADE_TURNING;
            currentMovementState = MovementState::Turning;

            //intentional fall-through; when completeMove() returns false, the PIDs and motor
            //outputs have not been updated, so go directly to the turn

        case MovementState::Turning:
            if (!completeTurn())
                return;

            //stop at the position
            zippy.stop();
            currentLinearVelocity = 0.0;
            currentAngularVelocity = 0.0;
            currentMovementState = MovementState::Stopped;
            break;
    }
    // */
}

bool DrivingController::completeMove()
{
    if (relativeShadowPosition.getD() > LINEAR_EPSILON &&
          stateDowngradeCounter)
    {
        stateDowngradeCounter--;
        executeMove();
        return false;
    }

    return true;
}

bool DrivingController::completeTurn()
{
    if (//abs(relativeShadowVelocity.atan2()) > ANGULAR_EPSILON &&
        stateDowngradeCounter)
    {
        stateDowngradeCounter--;

        // double factor = 1.0 + ((double)(MAX_STATE_DOWNGRADE_ITERATIONS - stateDowngradeCounter) /
            // (double)(MAX_STATE_DOWNGRADE_ITERATIONS));
        // double direction = relativeTargetVelocity->atan2();
        // if (driver.isReverseDirection())
            // direction = addAngles(direction, M_PI);
        // zippy.turn(relativeTargetPosition->getY(), factor * direction);
        // zippy.turn(relativeShadowPosition.getY(), factor * relativeShadowVelocity.atan2());

        zippy.turn(relativeShadowPosition.getY(),
            2.0 * subtractAngles(driver.getTargetPosition()->orientation.get(),
                sensors->getPosition()->orientation.get()));
        
        return false;
    }

    return true;
}

void DrivingController::executeMove()
{
    double linearVelocity, angularVelocity;
    if (driver.isReverseDirection()) {
        relativeShadowPosition.flip();
        relativeShadowVelocity.flip();
        pursuitController.executeMove(
            &relativeShadowPosition,
            &relativeShadowVelocity);
        linearVelocity = -pursuitController.getLinearVelocity();
        angularVelocity = pursuitController.getAngularVelocity();
    }
    else {
        pursuitController.executeMove(
            &relativeShadowPosition,
            &relativeShadowVelocity);
        linearVelocity = pursuitController.getLinearVelocity();
        angularVelocity = pursuitController.getAngularVelocity();
    }

    double limit = max(
        abs(currentLinearVelocity) * LIMIT_LINEAR_ACCELERATION,
        LIMIT_LINEAR_ACCELERATION_MIN);
    currentLinearVelocity += constrain(linearVelocity - currentLinearVelocity, -limit, limit);
    // currentLinearVelocity = constrain(linearVelocity, -LIMIT_LINEAR_VELOCITY_MAX, LIMIT_LINEAR_VELOCITY_MAX);
    // currentAngularVelocity += constrain(angularVelocity - currentAngularVelocity,
        // -currentAngularVelocity * LIMIT_LINEAR_ACCELERATION,
        // currentAngularVelocity * LIMIT_LINEAR_ACCELERATION);
    currentAngularVelocity = angularVelocity;
    zippy.move(currentLinearVelocity, currentAngularVelocity);
}

void DrivingController::stop()
{
    // SerialUSB.println("Stop");
    //unexpected stop    
    zippy.stop();
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    currentMovementState = MovementState::Stopped;
    currentDrivingState = DrivingState::MovingIntoPlace;
}
