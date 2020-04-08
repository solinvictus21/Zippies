
#include <Arduino.h>
#include "zippies/math/PID_v2.h"

PID_v2::PID_v2(int sps, double sp,
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

void PID_v2::start()
{
  previousInput = 0.0d;
  outputSum = 0.0d;
  isStarted = true;
}

double PID_v2::compute(double input)
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

void PID_v2::stop()
{
  isStarted = false;
}

void PID_v2::setTunings(double Kp, double Ki, double Kd)
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

void PID_v2::setOutputLimits(double Min, double Max)
{
  outMin = Min;
  outMax = Max;
}

void PID_v2::setReverseMode(bool rm)
{
   if (isReversed != rm) {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   isReversed = rm;
}
