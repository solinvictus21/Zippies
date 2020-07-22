
#include "zippies/controllers/RoutineController.h"
#include "zippies/ZippyRoutine.h"

#define INITIAL_MOVE_VELOCITY           200.0
#define INITIAL_MOVE_TIMING            3000

ZippyWaypoint moveIntoPlaceWaypoints[2]
{
    {     0.0,     0.0,     0.0, INITIAL_MOVE_TIMING },
    {     0.0,     0.0,     0.0 },
};

RoutineController::RoutineController(SensorFusor* s)
  : sensors(s)
{
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
    // reverseRoutine(defaultRoutines, defaultRoutinesCount);
    sensors->syncWithPreamble();
    executionState = RoutineExecutionState::PreSyncingWithPreamble;
}

void RoutineController::loop(unsigned long currentTime)
{
    switch (executionState) {
        case RoutineExecutionState::PreSyncingWithPreamble:
            if (sensors->foundPreamble()) {
                //plan the move to the starting position
                // SerialUSB.println("Found preamble. Moving into position.");
                const ZMatrix2* currentPosition = sensors->getPosition();
                moveIntoPlaceWaypoints[0].x = currentPosition->position.getX();
                moveIntoPlaceWaypoints[0].y = currentPosition->position.getY();
                moveIntoPlaceWaypoints[0].orientation = currentPosition->orientation.get();
                path.setKeyframes(&moveIntoPlaceWaypoints[0], 2);
                path.start(currentTime);
                executionState = RoutineExecutionState::MovingIntoPlace;
            }
            break;

        case RoutineExecutionState::MovingIntoPlace:
            path.interpolate(currentTime);
            pathFollowingController.followPath(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());

            if (path.isCompleted()) {
                //begin execution of the routine
                path.setKeyframes(defaultRoutines, defaultRoutinesCount);
                path.start(currentTime);
                executionState = RoutineExecutionState::Executing;
            }
            break;

        case RoutineExecutionState::Executing:
            path.interpolate(currentTime);
            pathFollowingController.followPath(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());
            
            if (path.isCompleted()) {
                //sync with preamble again before start of each routine execution
                sensors->syncWithPreamble();
                executionState = RoutineExecutionState::PostSyncingWithPreamble;
            }
            break;

        case RoutineExecutionState::PostSyncingWithPreamble:
            pathFollowingController.stopPath(
                sensors->getPosition(),
                path.getTargetPosition(),
                path.getTargetVelocity());

            if (sensors->foundPreamble()) {
                //restart the routine execution
                path.start(currentTime);
                executionState = RoutineExecutionState::Executing;
            }
            break;
    }
}

void RoutineController::stop()
{
    pathFollowingController.stop();
}
