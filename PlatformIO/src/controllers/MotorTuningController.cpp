
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/config/BodyConfig.h"

#define DEFAULT_TEST_TIME 1000
#define DEFAULT_MOTOR_POWER_DELTA 100.0
#define DEFAULT_ACCEPT_STRAIGHT_MOTION 200.0
// #define TEST_START_MOTOR_LEFT 7200.0
// #define TEST_START_MOTOR_RIGHT 9100.0
#define TEST_START_MOTOR_LEFT 6800.0
#define TEST_START_MOTOR_RIGHT 8800.0
#define TEST_SLOW_MOTION_THRESHOLD 100.0
#define EPSILON_LINEAR 1.0
#define EPSILON_ANGULAR 0.052359877559830

//power differential on orange is 9143 and 7156 = 0.121909319590159

MotorTuningController::MotorTuningController(SensorFusor* s)
  : sensors(s)
{
    leftMotorPower = TEST_START_MOTOR_LEFT;
    rightMotorPower = TEST_START_MOTOR_RIGHT;
}

void MotorTuningController::resetTest()
{
    testStartingPosition.set(sensors->getPosition());
    motors.setMotorsDirect(leftMotorPower, rightMotorPower);
    testTimer = DEFAULT_TEST_TIME;
}

void MotorTuningController::start()
{
    deltaPosition.reset();
    linearVelocity = 0.0;
    turnRadius = 0.0;
    powerDifferential = 0.0;
    testComplete = false;
    resetTest();

    display.clearScreen();
    displayMetrics();
    display.displayText(0, 5 * display.getFontHeight(), "STARTED");
}

bool MotorTuningController::loop(unsigned long deltaTime)
{
    if (testComplete)
        return true;

    if (deltaTime < testTimer) {
        testTimer -= deltaTime;
        return true;
    }
    testTimer = DEFAULT_TEST_TIME;
    display.displayText(0, 5 * display.getFontHeight(), "TESTING");

    //evaluate our motion during the previous test
    calculateMetrics();
    displayMetrics();

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

    return true;
}

bool MotorTuningController::ensureTestableMotion()
{
    //check if we're in motion
    if (deltaPosition.position.getD() < EPSILON_LINEAR) {
        //we're not moving at all; ramp up the power on both wheels
        leftMotorPower += DEFAULT_MOTOR_POWER_DELTA;
        rightMotorPower += DEFAULT_MOTOR_POWER_DELTA;
        resetTest();
        return false;
    }

    //check if we're turning
    double turn = deltaPosition.orientation.get();
    if (fabs(turn) >= EPSILON_ANGULAR) {
        //attempt to straighten our motion
        double powerDifference = 2.0 * powerDifferential * (leftMotorPower + rightMotorPower);
        leftMotorPower -= powerDifference;
        rightMotorPower += powerDifference;
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
        testComplete = true;
        display.displayText(0, 5 * display.getFontHeight(), "DIFF DONE");
    }
}

void MotorTuningController::testStiction()
{
    //now that we're moving straight, test how slow we can go
    if (deltaPosition.position.getD() > TEST_SLOW_MOTION_THRESHOLD) {
        leftMotorPower -= 5.0 * DEFAULT_MOTOR_POWER_DELTA;
        rightMotorPower -= 5.0 * DEFAULT_MOTOR_POWER_DELTA;
        resetTest();
        return;
    }

    motors.stopMotors();
    testComplete = true;
    display.displayText(0, 5 * display.getFontHeight(), "STIC DONE");
}

void MotorTuningController::testMinimumStraightSpeed()
{
}

void MotorTuningController::calculateMetrics()
{
    const ZMatrix2* currentPosition = sensors->getPosition();
    deltaPosition.set(currentPosition);
    deltaPosition.unconcatFrom(&testStartingPosition);

    double angularVelocity = deltaPosition.position.atan();
    if (angularVelocity == 0.0) {
        linearVelocity = deltaPosition.position.getY();
        turnRadius = 0.0;
        return;
    }

    linearVelocity =
        (deltaPosition.position.getD() * angularVelocity) /
        sin(angularVelocity);
    turnRadius = linearVelocity / angularVelocity;

    powerDifferential = 1.0 -
            ((fabs(turnRadius) - WHEEL_CENTER_BODY_CENTER_OFFSET) /
            (fabs(turnRadius) + WHEEL_CENTER_BODY_CENTER_OFFSET));
    if (turnRadius < 0.0)
        powerDifferential = -powerDifferential;
}

void MotorTuningController::displayMetrics()
{
    uint8_t y = 0;
    display.displayLabelAndData(0, 0, "R", (int)rightMotorPower);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, 0, "L", (int)leftMotorPower);

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "D", deltaPosition.position.getD());
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "O", RAD2DEG * deltaPosition.orientation.get());

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "V", (int)linearVelocity);
    display.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "R", (int)turnRadius);

    y += display.getFontHeight();
    display.displayLabelAndData(0, y, "P", 100.0 * powerDifferential);
}

void MotorTuningController::stop()
{
    motors.stopMotors();
    if (!testComplete)
        display.displayText(0, 5 * display.getFontHeight(), "STOPPED");
}