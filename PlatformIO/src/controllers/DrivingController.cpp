
#include "zippies/controllers/DrivingController.h"
#include "zippies/config/ZippyPathConfig.h"
#include "zippies/pursuit/PursuitController.h"

#define MOVE_TO_START_TIMING            5000
#define MOVE_TO_START_VELOCITY           300.0

DrivingController::DrivingController(SensorFusor* s)
  : sensors(s),
    pursuitController(s)
{
    getZippyRoutine(
        &waypoints,
        &timings,
        &commands,
        &commandCount);
}

void DrivingController::reset()
{
    //reset all the variables used to execute routines
    currentCommand = 0;
    currentTiming = 0;
    currentTimingEnd = 0;
    currentTimeRemaining = 0;
    isStopped = false;
    primaryAnchorPosition.reset();
    primaryDriver.reset();
    secondaryAnchorPosition.reset();
    secondaryDriver.reset();
    pursuitController.setReverseDirection(false);
    currentDrivingMode = DrivingMode::Move;
}

void DrivingController::start(unsigned long startTime)
{
    reset();
    previousTime = startTime;

    //setup the initial move to the starting position
    primaryDriver.setShadowPosition(sensors->getPosition());
    if (commands[0] == COMMAND_MOVE) {
        int waypointNum = commands[1];
        primaryDriver.setTargetPosition(
            waypoints[waypointNum].x, waypoints[waypointNum].y,
            DEG2RAD * waypoints[waypointNum].orientation);
        currentTimeRemaining = min(
            (unsigned long)(1000.0 * primaryDriver.getDistanceToTarget() / MOVE_TO_START_VELOCITY),
            MOVE_TO_START_TIMING);
        currentState = DrivingState::MovingIntoPlace;
    }
    else {
        primaryDriver.setTargetPosition(sensors->getPosition());
        currentTimeRemaining = 0;
        isStopped = true;
        currentState = DrivingState::HoldingAtStart;
    }

    moveIntoPlaceTimer = MOVE_TO_START_TIMING;
    // SerialUSB.println("Moving to starting point.");
}

void DrivingController::loop(unsigned long currentTime)
{
    unsigned long deltaTime = currentTime - previousTime;
    previousTime = currentTime;

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
        abs(primaryDriver.getLinearVelocity()) > primaryDriver.getDistanceToTarget())
    {
        //we've reached the starting position; hold there until the startup timer expires
        currentTimeRemaining = 0;
        primaryDriver.stop();
        isStopped = true;
        // SerialUSB.println("Holding at start.");
        currentState = DrivingState::HoldingAtStart;
        return deltaTime;
    }
    currentTimeRemaining -= deltaTime;
    moveIntoPlaceTimer -= deltaTime;

    //normal update
    primaryDriver.update(currentTimeRemaining);
    pursue(deltaTime);
    return 0;
}

unsigned long DrivingController::holdAtStart(unsigned long deltaTime)
{
    if (deltaTime > moveIntoPlaceTimer) {
        deltaTime -= moveIntoPlaceTimer;
        moveIntoPlaceTimer = 0;

        //sync with the preamble
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
        abs(primaryDriver.getLinearVelocity()) > primaryDriver.getDistanceToTarget())
    {
        //check if we have more commands to process
        if (!processCommands()) {
            //we've reached the end of the routine
            primaryDriver.stop();
            secondaryDriver.stop();
            isStopped = true;

            //sync with the preamble
            sensors->syncWithPreamble();
            currentState = DrivingState::SyncingWithPreamble;
            return deltaTime;
        }
    }
    currentTimeRemaining -= deltaTime;

    switch (currentDrivingMode)
    {
        case DrivingMode::Move:
            pursue(deltaTime);
            break;

        case DrivingMode::Watch:
            watch(deltaTime);
            break;

        case DrivingMode::Mimic:
            mimic(deltaTime);
            break;
    }

    return 0;
}

bool DrivingController::processCommands()
{
    isStopped = false;
    while (currentCommand < commandCount || currentTiming < currentTimingEnd)
    {
        if (currentTiming < currentTimingEnd) {
            initCurrentTiming();
            return true;
        }

        switch (commands[currentCommand]) {
            case COMMAND_MOVE:
            {
                currentCommand++;
                int waypointNum = commands[currentCommand];
                primaryAnchorPosition.set(
                    waypoints[waypointNum].x, waypoints[waypointNum].y,
                    DEG2RAD * waypoints[waypointNum].orientation);
                currentCommand++;
                currentDrivingMode = DrivingMode::Move;
                break;
            }

            case COMMAND_WATCH:
            {
                if (currentDrivingMode == DrivingMode::Move)
                    primaryDriver.stop();
                currentCommand++;
                int waypointNum = commands[currentCommand];
                secondaryAnchorPosition.set(
                    waypoints[waypointNum].x, waypoints[waypointNum].y,
                    DEG2RAD * waypoints[waypointNum].orientation);
                currentCommand++;
                currentDrivingMode = DrivingMode::Watch;
                break;
            }

            case COMMAND_MIMIC:
            {
                if (currentDrivingMode == DrivingMode::Move)
                    primaryDriver.stop();
                currentCommand++;
                int waypointNum = commands[currentCommand];
                secondaryAnchorPosition.set(
                    waypoints[waypointNum].x, waypoints[waypointNum].y,
                    DEG2RAD * waypoints[waypointNum].orientation);
                currentCommand++;
                currentDrivingMode = DrivingMode::Mimic;
                break;
            }

            case COMMAND_FORWARD:
                if (currentDrivingMode == DrivingMode::Move)
                    primaryDriver.setReverseDirection(false);
                else
                    secondaryDriver.setReverseDirection(false);
                pursuitController.setReverseDirection(false);
                currentCommand++;
                currentTiming = commands[currentCommand];
                currentCommand++;
                currentTimingEnd = currentTiming + commands[currentCommand];
                currentCommand++;
                break;

            case COMMAND_REVERSE:
                if (currentDrivingMode == DrivingMode::Move)
                    primaryDriver.setReverseDirection(true);
                else
                    secondaryDriver.setReverseDirection(true);
                pursuitController.setReverseDirection(true);
                currentCommand++;
                currentTiming = commands[currentCommand];
                currentCommand++;
                currentTimingEnd = currentTiming + commands[currentCommand];
                currentCommand++;
                break;

            case COMMAND_PAUSE:
                if (!isStopped) {
                    if (currentDrivingMode == DrivingMode::Move)
                        primaryDriver.stop();
                    else
                        secondaryDriver.stop();
                }
                currentCommand++;
                currentTimeRemaining += commands[currentCommand];
                isStopped = true;
                currentCommand++;
                return true;

        }
    }

    return false;
}

void DrivingController::initCurrentTiming()
{
    int waypointNum = timings[currentTiming].index;
    if (currentDrivingMode == DrivingMode::Move) {
        primaryDriver.setTargetPosition(
            waypoints[waypointNum].x, waypoints[waypointNum].y,
            DEG2RAD * waypoints[waypointNum].orientation);
    }
    else {
        secondaryDriver.setTargetPosition(
            waypoints[waypointNum].x, waypoints[waypointNum].y,
            DEG2RAD * waypoints[waypointNum].orientation);
    }
    currentTimeRemaining += timings[currentTiming].timing;

    currentTiming++;
}

void DrivingController::pursue(unsigned long deltaTime)
{
    if (!isStopped)
        primaryDriver.update(max(currentTimeRemaining, deltaTime));

    relativeShadowPosition.set(&primaryDriver.getShadowPosition()->position);
    relativeShadowPosition.rotate(&primaryAnchorPosition.orientation);
    relativeShadowPosition.add(&primaryAnchorPosition.position);

    //normal velocity calculation
    relativeShadowVelocity.set(primaryDriver.getShadowVelocity());
    relativeShadowVelocity.rotate(&primaryAnchorPosition.orientation);

    calculateRelativeTargets();
    if (!isStopped)
        pursuitController.continuePursuit(&relativeShadowPosition, &relativeShadowVelocity);
    else
        pursuitController.stopPursuit(&relativeShadowPosition, &relativeShadowVelocity);
}

void DrivingController::watch(unsigned long deltaTime)
{
    if (!isStopped)
        secondaryDriver.update(max(currentTimeRemaining, deltaTime));

    relativeShadowPosition.set(&primaryDriver.getTargetPosition()->position);
    relativeShadowPosition.rotate(&primaryAnchorPosition.orientation);
    relativeShadowPosition.add(&primaryAnchorPosition.position);

    //velocity is the direction from the current position to the secondary target position
    relativeShadowVelocity.set(&secondaryDriver.getShadowPosition()->position);
    relativeShadowVelocity.rotate(&secondaryAnchorPosition.orientation);
    relativeShadowVelocity.add(&secondaryAnchorPosition.position);
    relativeShadowVelocity.subtract(&relativeShadowPosition);

    calculateRelativeTargets();
    if (!isStopped)
        pursuitController.continueTurn(&relativeShadowPosition, &relativeShadowVelocity);
    else
        pursuitController.stopPursuit(&relativeShadowPosition, &relativeShadowVelocity);
}

void DrivingController::mimic(unsigned long deltaTime)
{
    if (!isStopped)
        secondaryDriver.update(max(currentTimeRemaining, deltaTime));

    relativeShadowPosition.set(&primaryDriver.getTargetPosition()->position);
    relativeShadowPosition.rotate(&primaryAnchorPosition.orientation);
    relativeShadowPosition.add(&primaryAnchorPosition.position);

    //velocity is the velocity of the secondary driver
    relativeShadowVelocity.set(secondaryDriver.getShadowVelocity());
    relativeShadowVelocity.rotate(&secondaryAnchorPosition.orientation);
    relativeShadowVelocity.unrotate(&primaryAnchorPosition.orientation);

    calculateRelativeTargets();
    if (!isStopped)
        pursuitController.continueTurn(&relativeShadowPosition, &relativeShadowVelocity);
    else
        pursuitController.stopPursuit(&relativeShadowPosition, &relativeShadowVelocity);
}

void DrivingController::calculateRelativeTargets()
{
    //make both relative to the current position
    const ZMatrix2* currentPosition = sensors->getPosition();
    relativeShadowPosition.subtract(&currentPosition->position);
    relativeShadowPosition.unrotate(&currentPosition->orientation);
    relativeShadowVelocity.unrotate(&currentPosition->orientation);
}

void DrivingController::stop()
{
    pursuitController.stop();
    currentState = DrivingState::MovingIntoPlace;
}
