
#include <Arduino.h>
#include "zippies/math/ZPID.h"

ZPID::ZPID(int sps, double sp,
        double Kp, double Ki, double Kd,
        double min, double max,
        bool pe, bool rm)
{
  samplesPerSecond = sps;
  setPoint = sp;
  outMin = min;
  outMax = max;
  pOnE = pe;
  isReversed = rm;
  setTunings(Kp, Ki, Kd);
}

void ZPID::start()
{
  previousInput = 0.0d;
  outputSum = 0.0d;
  isStarted = true;
}

double ZPID::compute(double input)
{
  if (!isStarted)
    return 0.0d;

  //compute all the working error variables
  double error = setPoint - input;
  double inputDelta = input - previousInput;
  previousInput = input;
  outputSum += ki * error;

  //add Proportional on Measurement if not in pOnE mode
  if (!pOnE)
    outputSum -= kp * inputDelta;

  outputSum = constrain(outputSum, outMin, outMax);

  //add Proportional on Error in pOnE mode
  double output;
  if (pOnE)
    output = kp * error;
  else
    output = 0;

  /*Compute Rest of PID Output*/
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
