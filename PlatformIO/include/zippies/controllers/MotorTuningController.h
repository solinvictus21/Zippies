
#ifndef _MOTORTUNINGCONTROLLER_H_
#define _MOTORTUNINGCONTROLLER_H_

#include "zippies/controllers/ZippyController.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/MotorDriver.h"
#include "zippies/hardware/ZippyFace.h"
#include "zippies/math/ZMatrix2.h"

typedef enum class _MotorTuningTestState
{
    StictionStart,
    StictionRamp,
    StictionLow,
    PowerDifferential,
    Stiction,
    MinimumStraightSpeed,
    Completed,
} MotorTuningTestState;

class MotorTuningController : public ZippyController
{

private:
    // SensorFusor* sensors;
    MotorDriver motors;
    ZippyFace display;
    uint8_t buttonsStatus = 0;

    MotorTuningTestState currentTestState = MotorTuningTestState::Stiction;
    double motorPowerTesting = 0.0;
    double stictionPowerTesting = 0.0;
    double currentMotorPower = 0.0;
    // double leftMotorPower = 0.0;
    // double rightMotorPower = 0.0;
    ZMatrix2 testStartingPosition;
    unsigned long testTimer = 0;

    //test metrics
    ZMatrix2 deltaPosition;
    double linearVelocity = 0.0;
    double linearVelocityLeft = 0.0;
    double linearVelocityRight = 0.0;
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
    // MotorTuningController(const ZippyConfig* zc, SensorFusor* sensors);
    MotorTuningController(const ZippyConfig* zc);

    bool start();
    bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros);
    void stop();

    ~MotorTuningController() {}

};

#endif