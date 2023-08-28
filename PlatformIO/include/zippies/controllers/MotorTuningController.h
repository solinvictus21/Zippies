
#ifndef _MOTORTUNINGCONTROLLER_H_
#define _MOTORTUNINGCONTROLLER_H_

#include "zippies/controllers/ZippyController.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/MotorDriver.h"
#include "zippies/hardware/ZippyFace.h"
#include "zippies/math/ZMatrix2.h"

typedef enum class _MotorTuningTestState
{
    PowerDifferential,
    Stiction,
    MinimumStraightSpeed,
} MotorTuningTestState;

class MotorTuningController : public ZippyController
{

private:
    SensorFusor* sensors;
    MotorDriver motors;
    ZippyFace display;

    MotorTuningTestState currentTestState = MotorTuningTestState::Stiction;
    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;
    ZMatrix2 testStartingPosition;
    unsigned long testTimer = 0;
    bool testComplete = false;

    //test metrics
    ZMatrix2 deltaPosition;
    double linearVelocity = 0.0;
    double turnRadius = 0.0;
    double powerDifferential = 0.0;

    void resetTest();
    bool ensureTestableMotion();
    void testPowerDifferential();
    void testStiction();
    void testMinimumStraightSpeed();
    void calculateMetrics();
    void displayMetrics();

public:
    MotorTuningController(SensorFusor* sensors);

    void start();
    bool loop(unsigned long deltaTime);
    void stop();

    ~MotorTuningController() {}

};

#endif