
#include <Arduino.h>
#include "zippies/math/ZPID.h"

ZPID::ZPID(int sps, //double sp,
    double Kp, double Ki, double Kd,
    double min, double max,
    bool pm, bool rm)
{
    samplesPerSecond = sps;
    // setPoint = sp;
    outMin = min;
    outMax = max;
    proportionalOnMeasurement = pm;
    isReversed = rm;
    setTunings(Kp, Ki, Kd);
}

void ZPID::start()
{
    previousInput = 0.0;
    outputSum = 0.0;
    isStarted = true;
}

double ZPID::compute(double input, double setPoint)
{
    if (!isStarted)
        return 0.0;

    //compute all the working error variables
    double error = setPoint - input;
    double inputDelta = input - previousInput;
    previousInput = input;
    outputSum += ki * error;

    //add Proportional on Measurement if not in pOnE mode
    if (proportionalOnMeasurement)
        outputSum -= kp * inputDelta;

    outputSum = constrain(outputSum, outMin, outMax);

    //add Proportional on Error in pOnE mode
    double output;
    if (proportionalOnMeasurement)
        output = 0;
    else
        output = kp * error;

    //compute the rest of the PID output
    output += outputSum - (kd * inputDelta);
    return constrain(output, outMin, outMax);
}

void ZPID::stop()
{
    isStarted = false;
}

void ZPID::setTunings(double Kp, double Ki, double Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    kp = Kp;
    ki = Ki / ((double)samplesPerSecond);
    kd = Kd * ((double)samplesPerSecond);

    if (isReversed) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void ZPID::setOutputLimits(double Min, double Max)
{
    outMin = Min;
    outMax = Max;
}

void ZPID::setReverseMode(bool rm)
{
    if (isReversed != rm) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    isReversed = rm;
}
