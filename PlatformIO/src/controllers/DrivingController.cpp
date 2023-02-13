
#include "zippies/controllers/DrivingController.h"
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/pursuit/PursuitController.h"
#include "zippies/ZippyMath.h"

#define MOVE_TO_START_TIMING            5000
#define MOVE_TO_START_VELOCITY           300.0

/*
ZMatrix2 startingPosition;
ZMatrix2 targetStartingPosition;
ZVector2 startToTargetVelocity;
*/

DrivingController::DrivingController(SensorFusor* s)
  : sensors(s),
    pursuitController(s)
{
    routineData = getZippyRoutineData();
}

void DrivingController::reset()
{
    //reset all the variables used to execute routines
    isStopped = false;
    currentCommand = 0;
    currentTiming = 0;
    currentTimeRemaining = 0;
    driver.reset();
}

void DrivingController::start()
{
    reset();

    //setup the initial move to the starting position
    driver.start(sensors->getPosition(),
        routineData->startingPosition[0],
        routineData->startingPosition[1],
        routineData->startingPosition[2] * DEG2RAD);
   
    /*
    SerialUSB.print("Starting move into place: ");    
    SerialUSB.print(routineData->startingPosition[0]);
    SerialUSB.print(", ");
    SerialUSB.print(routineData->startingPosition[1]);
    SerialUSB.print(", ");
    SerialUSB.println(routineData->startingPosition[2]);
    */

    //drive to the start position
    currentTimeRemaining = min(
        (unsigned long)(1000.0 * driver.getDistanceToTarget() / MOVE_TO_START_VELOCITY),
        (unsigned long)MOVE_TO_START_TIMING);
    moveIntoPlaceTimer = MOVE_TO_START_TIMING;
    currentState = DrivingState::MovingIntoPlace;

    // SerialUSB.print("Moving to starting position: ");
    // SerialUSB.println(currentTimeRemaining);
}

void DrivingController::loop(unsigned long deltaTime)
{
    while (deltaTime) {
        switch (currentState) {
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
    if (deltaTime > currentTimeRemaining ||
        abs(driver.getLinearVelocity()) > driver.getDistanceToTarget())
    {
        //we've reached the starting position
        currentTimeRemaining = 0;
        driver.stop();
        isStopped = true;

        //hold at the starting position until the startup timer expires
        // SerialUSB.println("Holding at start.");
        currentState = DrivingState::HoldingAtStart;
        return deltaTime;
    }
    currentTimeRemaining -= deltaTime;
    moveIntoPlaceTimer -= deltaTime;

    //normal update
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::holdAtStart(unsigned long deltaTime)
{
    /*
    SerialUSB.print("Holding: ");
    SerialUSB.print(moveIntoPlaceTimer);
    SerialUSB.print(" deltaTime: ");
    SerialUSB.println(deltaTime);
    */

    if (deltaTime > moveIntoPlaceTimer) {
        deltaTime -= moveIntoPlaceTimer;
        moveIntoPlaceTimer = 0;

        //sync with the preamble
        // SerialUSB.println("Syncing with preamble.");
        sensors->syncWithPreamble();
        currentState = DrivingState::SyncingWithPreamble;
        return deltaTime;
    }
    moveIntoPlaceTimer -= deltaTime;

    //continue waiting
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::syncWithPreamble(unsigned long deltaTime)
{
    // SerialUSB.println("Holding for preamble.");
    if (sensors->foundPreamble()) {
        // SerialUSB.println("Found preamble");
        //begin execution of the routine
        reset();

        //setup the first command
        processCommands();

        //start executing the routine
        // SerialUSB.println("Starting execution.");
        currentState = DrivingState::Executing;
        return deltaTime;
    }

    //continue waiting
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::execute(unsigned long deltaTime)
{
    if (deltaTime > currentTimeRemaining ||
        abs(driver.getLinearVelocity()) > driver.getDistanceToTarget())
    {
        //check if we have more commands to process
        if (!processCommands()) {
            //we've reached the end of the routine
            driver.stop();
            isStopped = true;

            //sync with the preamble
            sensors->syncWithPreamble();
            currentState = DrivingState::SyncingWithPreamble;
            return deltaTime;
        }
    }
    currentTimeRemaining -= deltaTime;

    pursue(deltaTime);
    return 0;
}

bool DrivingController::processCommands()
{
    if (currentCommand >= routineData->routineMovementCount)
        return false;

    int timing = routineData->routineMovements[currentCommand++];

    //increment the timing
    currentTimeRemaining += abs(timing);
    if (timing < 0) {
        isStopped = true;
        return true;
    }
    isStopped = false;

    //pull the waypoint
    driver.setTargetPosition(
        routineData->routineMovements[currentCommand],
        routineData->routineMovements[currentCommand+1],
        routineData->routineMovements[currentCommand+2] * DEG2RAD);
    currentCommand += 3;

    return true;
}

void DrivingController::pursue(unsigned long deltaTime)
{
    if (!isStopped)
        driver.update(max(currentTimeRemaining, deltaTime));

    if (currentState == DrivingState::Executing) {
        SerialUSB.print("absolute target: ");
        driver.getShadowPosition()->printDebug();
    }

    //calculate vectors relative to current position
    const ZMatrix2* currentPosition = sensors->getPosition();
    relativeShadowPosition.set(&driver.getShadowPosition()->position);
    relativeShadowPosition.subtract(&currentPosition->position);
    relativeShadowPosition.unrotate(&currentPosition->orientation);
    relativeShadowVelocity.set(driver.getShadowVelocity());
    relativeShadowVelocity.unrotate(&currentPosition->orientation);


    if (!isStopped)
        pursuitController.continuePursuit(&relativeShadowPosition, &relativeShadowVelocity, driver.isReverseDirection());
    else
        pursuitController.stopPursuit(&relativeShadowPosition, &relativeShadowVelocity, driver.isReverseDirection());
}

void DrivingController::stop()
{
    pursuitController.stop();
    currentState = DrivingState::MovingIntoPlace;
}
