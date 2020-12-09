
#include "zippies/controllers/RoutineController.h"
// #include "zippies/pursuit/PurePursuitController.h"
#include "zippies/pursuit/ArcPursuitController.h"
#include "zippies/ZippyRoutine.h"

#define INITIAL_MOVE_VELOCITY           200.0
#define INITIAL_MOVE_TIMING            2000

ZippyWaypoint moveIntoPlaceWaypoints[2]
{
    {     0.0,     0.0,     0.0, INITIAL_MOVE_TIMING },
    {     0.0,     0.0,     0.0 },
};

RoutineController::RoutineController(SensorFusor* s)
  : sensors(s)
{
    // pursuitController = new PurePursuitController();
    pursuitController = new ArcPursuitController();
    getZippyRoutine(
        &defaultRoutinesStartPosition,
        &defaultRoutines,
        &defaultRoutinesCount);
    
    moveIntoPlaceWaypoints[1].x = defaultRoutinesStartPosition.position.getX();
    moveIntoPlaceWaypoints[1].y = defaultRoutinesStartPosition.position.getY();
    moveIntoPlaceWaypoints[1].orientation = defaultRoutinesStartPosition.orientation.get();
}

void RoutineController::start(unsigned long currentTime)
{
    /*
    sensors->syncWithPreamble();
    executionState = RoutineExecutionState::PreSyncingWithPreamble;
    */
    const ZMatrix2* currentPosition = sensors->getPosition();
    moveIntoPlaceWaypoints[0].x = currentPosition->position.getX();
    moveIntoPlaceWaypoints[0].y = currentPosition->position.getY();
    moveIntoPlaceWaypoints[0].orientation = currentPosition->orientation.get();
    path.setKeyframes(&moveIntoPlaceWaypoints[0], 2);
    path.start(currentTime);
    executionState = RoutineExecutionState::MovingIntoPlace;
}

void RoutineController::loop(unsigned long currentTime)
{
    /*
    if (executionState == RoutineExecutionState::PreSyncingWithPreamble) {
        if (!sensors->foundPreamble())
            return;

        const ZMatrix2* currentPosition = sensors->getPosition();
        moveIntoPlaceWaypoints[0].x = currentPosition->position.getX();
        moveIntoPlaceWaypoints[0].y = currentPosition->position.getY();
        moveIntoPlaceWaypoints[0].orientation = currentPosition->orientation.get();
        path.setKeyframes(&moveIntoPlaceWaypoints[0], 2);
        path.start(currentTime);
        executionState = RoutineExecutionState::MovingIntoPlace;
    }
    */

    switch (executionState) {
        case RoutineExecutionState::MovingIntoPlace:
            path.interpolate(currentTime);
            pursuitController->continuePursuit(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());

            if (path.isCompleted()) {
                /*
                //begin execution of the routine
                path.setKeyframes(defaultRoutines, defaultRoutinesCount);
                path.start(currentTime);
                executionState = RoutineExecutionState::Executing;
                */
               //sync with the preamble
               sensors->syncWithPreamble();
               executionState = RoutineExecutionState::SyncingWithPreamble;
            }
            break;

        case RoutineExecutionState::SyncingWithPreamble:
            pursuitController->stopPursuit(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());

            if (sensors->foundPreamble()) {
                //begin execution of the routine
                path.setKeyframes(defaultRoutines, defaultRoutinesCount);
                path.start(currentTime);
                executionState = RoutineExecutionState::Executing;
            }
            break;

        case RoutineExecutionState::Executing:
            path.interpolate(currentTime);
            pursuitController->continuePursuit(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());
            
            if (path.isCompleted()) {
                //sync with preamble again before start of each routine execution
                sensors->syncWithPreamble();
                executionState = RoutineExecutionState::SyncingWithPreamble;
            }
            break;
    }
}

void RoutineController::stop()
{
    pursuitController->stop();
}
