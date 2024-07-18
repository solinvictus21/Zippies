
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/config/BodyConfig.h"

#define DEFAULT_TEST_TIME 1000
#define DEFAULT_MOTOR_POWER_DELTA 20.0
#define DEFAULT_ACCEPT_STRAIGHT_MOTION 200.0
// #define TEST_START_MOTOR_LEFT 3600.0
// #define TEST_START_MOTOR_RIGHT 5000.0
#define TEST_SLOW_MOTION_THRESHOLD 100.0
#define EPSILON_LINEAR 1.0
#define EPSILON_ANGULAR 0.052359877559830

#define TEST_START_MOTOR          3600.0
#define TEST_STICTION_START_TIME    17
#define TEST_STICTION_START_DELTA 4000.0
#define TEST_STICTION_DELTA        100.0

#define BUTTON_POWER_DELTA 100.0

//power differential on orange is 9143 and 7156 = 0.121909319590159

MotorTuningController::MotorTuningController(const ZippyConfig* zc)
  : motors(zc)
{
    motorPowerTesting = TEST_START_MOTOR;
    stictionPowerTesting = motorPowerTesting + TEST_STICTION_START_DELTA;
    // leftMotorPower = TEST_START_MOTOR_LEFT;
    // rightMotorPower = TEST_START_MOTOR_RIGHT;
    buttonsStatus = display.getButtons();
}

void MotorTuningController::resetTest()
{
    // testStartingPosition.set(sensors->getPosition());
    // motors.setMotorsDirect(leftMotorPower, rightMotorPower);
    // testTimer = DEFAULT_TEST_TIME;
    currentMotorPower = 0.0;
    motors.stopMotors();
    testTimer = DEFAULT_TEST_TIME;
    currentTestState = MotorTuningTestState::StictionStart;
    displayMetrics();
}

bool MotorTuningController::start()
{
    deltaPosition.reset();
    linearVelocity = 0.0;
    linearVelocityLeft = 0.0;
    linearVelocityRight = 0.0;
    turnRadius = 0.0;
    powerDifferential = 0.0;
    resetTest();

    display.clearScreen();
    displayMetrics();

    currentMotorPower = 0.0;
    motors.stopMotors();
    testTimer = DEFAULT_TEST_TIME;
    currentTestState = MotorTuningTestState::StictionStart;
    display.displayText(0, 5 * display.getFontHeight(), "STARTED");

    return true;
}

bool MotorTuningController::loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    // /*
    //process button presses
    uint8_t newButtonsStatus = display.getButtons();
    uint8_t buttonsJustPressed = (buttonsStatus ^ newButtonsStatus) & newButtonsStatus;
    buttonsStatus = newButtonsStatus;
    if (buttonsJustPressed & TSButtonUpperLeft) {
        motorPowerTesting += BUTTON_POWER_DELTA;
        resetTest();
        return true;
    }

    if (buttonsJustPressed & TSButtonLowerLeft) {
        motorPowerTesting -= BUTTON_POWER_DELTA;
        resetTest();
        return true;
    }

    if (buttonsJustPressed & TSButtonUpperRight) {
        stictionPowerTesting += TEST_STICTION_DELTA;
        resetTest();
        return true;
    }

    if (buttonsJustPressed & TSButtonLowerRight) {
        stictionPowerTesting -= TEST_STICTION_DELTA;
        resetTest();
        return true;
    }

    if (currentTestState == MotorTuningTestState::Completed)
        return true;

    if (deltaTimeMicros / 1000 < testTimer) {
        testTimer -= deltaTimeMicros / 1000;
        return true;
    }
    testTimer = DEFAULT_TEST_TIME;
    display.displayText(0, 5 * display.getFontHeight(), "TESTING");

    //evaluate our motion during the previous test
    calculateMetrics();
    displayMetrics();

    switch(currentTestState) {
        case MotorTuningTestState::StictionStart:
            currentMotorPower = stictionPowerTesting;
            motors.setMotorsDirect(currentMotorPower, currentMotorPower);
            testTimer = TEST_STICTION_START_TIME;
            currentTestState = MotorTuningTestState::StictionRamp;
            displayMetrics();
            break;
        case MotorTuningTestState::StictionRamp:
            currentMotorPower = motorPowerTesting;
            motors.setMotorsDirect(currentMotorPower, currentMotorPower);
            testTimer = 5 * DEFAULT_TEST_TIME;
            currentTestState = MotorTuningTestState::StictionLow;
            displayMetrics();
            break;
        case MotorTuningTestState::StictionLow:
            currentMotorPower = 0.0;
            motors.stopMotors();
            testTimer = DEFAULT_TEST_TIME;
            currentTestState = MotorTuningTestState::Completed;
            displayMetrics();
            break;
    }
    // */

    /*
    if (ensureTestableMotion()) {
        switch(currentTestState) {
            case MotorTuningTestState::PowerDifferential:
                testPowerDifferential();
                break;
            case MotorTuningTestState::Stiction:
                testStiction();
                break;
            case MotorTuningTestState::MinimumStraightSpeed:
                testMinimumStraightSpeed();
                break;
        }
    }
    */

    return true;
}

bool MotorTuningController::ensureTestableMotion()
{
    //check if we're in motion
    if (deltaPosition.position.getD() < EPSILON_LINEAR) {
        //we're not moving at all; ramp up the power on both wheels
        // leftMotorPower += DEFAULT_MOTOR_POWER_DELTA;
        // rightMotorPower += DEFAULT_MOTOR_POWER_DELTA;
        resetTest();
        return false;
    }

    //check if we're turning
    double turn = deltaPosition.orientation.get();
    if (fabs(turn) >= EPSILON_ANGULAR) {
        //attempt to straighten our motion
        // double powerDifference = 2.0 * powerDifferential * (leftMotorPower + rightMotorPower);
        // leftMotorPower -= powerDifference;
        // rightMotorPower += powerDifference;
        resetTest();
        return false;
    }

    return true;
}

void MotorTuningController::testPowerDifferential()
{
    //continue moving long enough to prove that we're moving straight
    if (deltaPosition.position.getD() > DEFAULT_ACCEPT_STRAIGHT_MOTION) {
        motors.stopMotors();
        currentTestState = MotorTuningTestState::Completed;
        display.displayText(0, 5 * display.getFontHeight(), "DIFF DONE");
    }
}

void MotorTuningController::testStiction()
{
    //now that we're moving straight, test how slow we can go
    if (deltaPosition.position.getD() > TEST_SLOW_MOTION_THRESHOLD) {
        // leftMotorPower -= 5.0 * DEFAULT_MOTOR_POWER_DELTA;
        // rightMotorPower -= 5.0 * DEFAULT_MOTOR_POWER_DELTA;
        resetTest();
        return;
    }

    motors.stopMotors();
    currentTestState = MotorTuningTestState::Completed;
    display.displayText(0, 5 * display.getFontHeight(), "STIC DONE");
}

void MotorTuningController::testMinimumStraightSpeed()
{
}

void MotorTuningController::calculateMetrics()
{
    /*
    const ZMatrix2* currentPosition = sensors->getPosition();
    deltaPosition.set(currentPosition);
    deltaPosition.unconcatFrom(&testStartingPosition);

    double angularVelocity = deltaPosition.orientation.get();
    if (fabs(angularVelocity) < EPSILON_ANGULAR) {
        turnRadius = 0.0;
        linearVelocity = deltaPosition.position.getY();
        linearVelocityLeft = deltaPosition.position.getY();
        linearVelocityRight = deltaPosition.position.getY();
        powerDifferential = 0.0;
        return;
    }

    turnRadius = deltaPosition.position.getD() / (2.0 * sin(deltaPosition.orientation.get() / 2.0));
    linearVelocity = turnRadius * angularVelocity;
    linearVelocityLeft = angularVelocity * (fabs(turnRadius) - WHEEL_CENTER_BODY_CENTER_OFFSET);
    linearVelocityRight = angularVelocity * (fabs(turnRadius) + WHEEL_CENTER_BODY_CENTER_OFFSET);

    powerDifferential = (linearVelocityRight - linearVelocityLeft) / (fabs(linearVelocityLeft) + fabs(linearVelocityRight));
    if (turnRadius < 0.0)
        powerDifferential = -powerDifferential;
    */
}

void MotorTuningController::displayMetrics()
{
    uint8_t y = 0;
    display.displayLabelAndData(0, y, "P", (int)motorPowerTesting);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "S", (int)(stictionPowerTesting));

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "C", (int)currentMotorPower);
    /*
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "O", RAD2DEG * deltaPosition.position.atan());

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "L", linearVelocity);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "A", RAD2DEG * deltaPosition.orientation.get());

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "J", linearVelocityRight);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "K", linearVelocityLeft);

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "Q", turnRadius);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "R", 100.0 * powerDifferential);
    */
}

/*
void MotorTuningController::displayMetrics()
{
    uint8_t y = 0;
    display.displayLabelAndData(0, y, "R", (int)rightMotorPower);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "L", (int)leftMotorPower);

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "D", deltaPosition.position.getD());
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "O", RAD2DEG * deltaPosition.position.atan());

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "L", linearVelocity);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "A", RAD2DEG * deltaPosition.orientation.get());

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "J", linearVelocityRight);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "K", linearVelocityLeft);

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "Q", turnRadius);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "R", 100.0 * powerDifferential);
}
*/

void MotorTuningController::stop()
{
    motors.stopMotors();
    if (currentTestState != MotorTuningTestState::Completed)
        display.displayText(0, 5 * display.getFontHeight(), "STOPPED");
}