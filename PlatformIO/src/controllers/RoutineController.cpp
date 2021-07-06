
#include "zippies/controllers/RoutineController.h"
#include "zippies/pursuit/PurePursuitController.h"
// #include "zippies/pursuit/ArcPursuitController.h"
#include "zippies/pursuit/ScissorPursuitController.h"
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
    pursuitController = new ScissorPursuitController(s);
    /*
    getZippyRoutine(
        &defaultRoutinesStartPosition,
        &defaultRoutines,
        &defaultRoutinesCount);
    */
    
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

void RoutineController::calculateRelativeTargets()
{
    const ZMatrix2* currentPosition = sensors->getPosition();
    relativeTargetPosition.set(path.getTargetPosition());
    relativeTargetPosition.subtractVector(&currentPosition->position);
    relativeTargetPosition.unrotate(&currentPosition->orientation);
    relativeTargetVelocity.set(path.getTargetVelocity());
    relativeTargetVelocity.unrotate(&currentPosition->orientation);

    /*
    // SerialUSB.print("tx=");
    // SerialUSB.print(path.getTargetPosition()->getX());
    // SerialUSB.print(" ty=");
    // SerialUSB.print(path.getTargetPosition()->getY());
    SerialUSB.print(" vx=");
    SerialUSB.print(path.getTargetVelocity()->getX());
    SerialUSB.print(" vy=");
    SerialUSB.print(path.getTargetVelocity()->getY());

    static ZVector2 previousTargetPosition;
    SerialUSB.print(" td=");
    SerialUSB.print(sqrt(
        pow(path.getTargetPosition()->getX() - previousTargetPosition.getX(), 2.0) +
        pow(path.getTargetPosition()->getY() - previousTargetPosition.getY(), 2.0)));
    SerialUSB.print(" vd=");
    SerialUSB.println(path.getTargetVelocity()->getD());
    previousTargetPosition.set(path.getTargetPosition());
    */
}

void RoutineController::loop(unsigned long currentTime)
{
    switch (executionState) {
        case RoutineExecutionState::MovingIntoPlace:
            path.interpolate(currentTime);
            calculateRelativeTargets();
            pursuitController->continuePursuit(&relativeTargetPosition, &relativeTargetVelocity);

            if (path.isCompleted()) {
               path.setKeyframes(defaultRoutines, defaultRoutinesCount);
               //sync with the preamble
               sensors->syncWithPreamble();
               executionState = RoutineExecutionState::SyncingWithPreamble;
            }
            break;

        case RoutineExecutionState::SyncingWithPreamble:
            calculateRelativeTargets();
            pursuitController->stopPursuit(&relativeTargetPosition, &relativeTargetVelocity);

            if (sensors->foundPreamble()) {
                //begin execution of the routine
                path.start(currentTime);
                executionState = RoutineExecutionState::Executing;
            }
            break;

        case RoutineExecutionState::Executing:
            path.interpolate(currentTime);
            calculateRelativeTargets();
            pursuitController->continuePursuit(&relativeTargetPosition, &relativeTargetVelocity);
            
            if (path.isCompleted()) {
                // SerialUSB.println("PATH STOPPED!!!");
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
