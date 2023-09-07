
#ifdef WEBOTS_SUPPORT
#include <webots/Robot.hpp>
using namespace webots;
#endif

#include "zippies/controllers/DrivingController.h"
#include "zippies/pursuit/PursuitController.h"
#include "zippies/ZippyMath.h"

#define MOVE_TO_START_TIMING                       5000
#define MOVE_TO_START_VELOCITY                      300.0

#define STATE_DOWNGRADE_COUNT                        30

//5 millimeters
#define LINEAR_EPSILON                                0.50

//3 degrees
#define ANGULAR_EPSILON                               0.052359877559830

//#define LIMIT_LINEAR_ACCELERATION_MIN                30.0
//#define LIMIT_LINEAR_ACCELERATION                     0.5
//#define LIMIT_LINEAR_VELOCITY_MAX                    50.0

#ifdef WEBOTS_SUPPORT
DrivingController::DrivingController(const ZippyConfig* zc, SensorFusor* s, Robot* zw, Node* zippyShadow)
  : sensors(s),
    zippy(zc, zw)
{
    zippyShadowTranslation = zippyShadow->getField("translation");
    zippyShadowRotation = zippyShadow->getField("rotation");
    const double* currentValues = zippyShadowTranslation->getSFVec3f();
    translationValues[2] = currentValues[2];
    routineData = &zc->routine[0];
}
#else
DrivingController::DrivingController(const ZippyConfig* zc, SensorFusor* s)
  : sensors(s),
    zippy(zc)
{
    routineData = &zc->routine[0];
}
#endif

void DrivingController::start()
{
    //this function is only called when the Zippy loses and then reacquires the LIghthouse signal
    //in this scenario, we need to do a full restart of the driving capabilities
    resetRoutine();
    driver.reset();

    //setup the initial move to the starting position
    double distanceToStart = sqrt(
        pow(routineData->startingPosition[0] - sensors->getPosition()->position.getX(), 2.0) +
        pow(routineData->startingPosition[1] - sensors->getPosition()->position.getY(), 2.0));
    driver.start(sensors->getPosition(),
        routineData->startingPosition[0],
        routineData->startingPosition[1],
        routineData->startingPosition[2] * DEG2RAD,
        fmin((unsigned long)(1000.0 * distanceToStart / MOVE_TO_START_VELOCITY),
             (unsigned long)MOVE_TO_START_TIMING));

    //setup the timings to drive to the start position
    currentTimeRemaining = MOVE_TO_START_TIMING;
    currentDrivingState = DrivingState::MovingIntoPlace;
}

void DrivingController::resetRoutine()
{
    //reset all the variables used to execute routines
    currentCommand = 0;
    currentTimeRemaining = 0;
}

bool DrivingController::loop(unsigned long deltaTime)
{
    //this function is invoked by the outer loop whenever a new position has become available from the Lighthouse
    while (deltaTime) {
        switch (currentDrivingState) {
            case DrivingState::MovingIntoPlace:
                deltaTime = moveIntoPlace(deltaTime);
                break;

            case DrivingState::SyncingWithPreamble:
                deltaTime = syncWithPreamble(deltaTime);
                break;

            case DrivingState::Executing:
                deltaTime = execute(deltaTime);
                break;
        }
    }
    return true;
}

unsigned long DrivingController::moveIntoPlace(unsigned long deltaTime)
{
    if (deltaTime > currentTimeRemaining) {
        //hold at the starting position until the startup timer expires
        sensors->syncWithPreamble();
        currentDrivingState = DrivingState::SyncingWithPreamble;
        return deltaTime;
    }

    currentTimeRemaining -= deltaTime;
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
    if (deltaTime >= currentTimeRemaining) {
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
        return false;
    }

    //increment the timing
    int timing = routineData->routineMovements[currentCommand++];
    if (timing < 0) {
        //a negative timing indicates that we're temporarily stopping at this position
        currentTimeRemaining += -timing;
        return true;
    }

    //pull the waypoint
    currentTimeRemaining += timing;
    driver.setTargetPosition(
        routineData->routineMovements[currentCommand],
        routineData->routineMovements[currentCommand+1],
        routineData->routineMovements[currentCommand+2] * DEG2RAD,
        timing);
    currentCommand += 3;

    //just in case we were in the middle of a state downgrade, reset the downgrade counter
    stateDowngradeCounter = STATE_DOWNGRADE_COUNT;

    return true;
}

void DrivingController::pursue(unsigned long deltaTime)
{
//    driver.update(fmax(currentTimeRemaining, deltaTime));
    driver.loop(deltaTime);

#ifdef WEBOTS_SUPPORT
    const ZVector2* shadowPosition = driver.getShadowPosition();
    translationValues[0] = shadowPosition->getX() / 1000.0;
    translationValues[1] = shadowPosition->getY() / 1000.0;
    zippyShadowTranslation->setSFVec3f(translationValues);
    const ZVector2* shadowVelocity = driver.getShadowVelocity();
    rotationValues[3] = -shadowVelocity->atan2();
    zippyShadowRotation->setSFRotation(rotationValues);
#endif

    zippy.setInput(sensors->getPositionDelta());

    //calculate vectors relative to current position
    const ZMatrix2* currentPosition = sensors->getPosition();
    relativeShadowPosition.set(driver.getShadowPosition());
    relativeShadowPosition.subtract(&currentPosition->position);
    relativeShadowPosition.unrotate(&currentPosition->orientation);
    relativeShadowVelocity.set(driver.getShadowVelocity());
    relativeShadowVelocity.unrotate(&currentPosition->orientation);

    if (!driver.isHoldingPosition()) {
        if (!isMoving) {
            //start moving
            zippy.start();
            stateDowngradeCounter = STATE_DOWNGRADE_COUNT;
            isMoving = true;
        }
        executeMove();
    }
    else if (isMoving && completeMove()) {
        //stop at the current position
        zippy.stop();
        currentLinearVelocity = 0.0;
        currentAngularVelocity = 0.0;
        isMoving = false;
    }
}

void DrivingController::executeMove()
{
    if (driver.isReverseDirection()) {
        relativeShadowPosition.flip();
        relativeShadowVelocity.flip();
        pursuitController.executeMove(
            &relativeShadowPosition,
            &relativeShadowVelocity);
        currentLinearVelocity = -pursuitController.getLinearVelocity();
        currentAngularVelocity = pursuitController.getAngularVelocity();
    }
    else {
        pursuitController.executeMove(
            &relativeShadowPosition,
            &relativeShadowVelocity);
        currentLinearVelocity = pursuitController.getLinearVelocity();
        currentAngularVelocity = pursuitController.getAngularVelocity();
    }

    /*
    double limit = fmax(
        abs(currentLinearVelocity) * LIMIT_LINEAR_ACCELERATION,
        LIMIT_LINEAR_ACCELERATION_MIN);
    currentLinearVelocity += constrain(linearVelocity - currentLinearVelocity, -limit, limit);
     */
    zippy.move(currentLinearVelocity, currentAngularVelocity);
}

bool DrivingController::completeMove()
{
    if (stateDowngradeCounter &&
        (relativeShadowPosition.getD() > LINEAR_EPSILON || relativeShadowVelocity.atan2() < ANGULAR_EPSILON))
    {
        stateDowngradeCounter--;
        //while completing a move (preparing to stop at the current target position), we prioritize ensuring
        //that we line up with the target position along our X axis and turn toward the velocity orientation
        //only insomuch as that will get us closer to our final target orientation
        zippy.move(relativeShadowPosition.getY(),
                   relativeShadowVelocity.atan2());
        return false;
    }

    return true;
}

void DrivingController::stop()
{
    // SerialUSB.println("Stop");
    //unexpected stop    
    zippy.stop();
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
    isMoving = false;
    currentDrivingState = DrivingState::MovingIntoPlace;
}
