
#ifndef _ERRORACCUMULATOR_H_
#define _ERRORACCUMULATOR_H_

#include <Arduino.h>

class StatisticsAccumulator
{

private:
  double valueA = 0.0d;
  double valueM = 0.0d;
  double valueS = 0.0d;
  int sampleCounter = 0;

public:
  StatisticsAccumulator()
  {}

  void reset()
  {
    valueA = 0.0d;
    valueM = 0.0d;
    valueS = 0.0d;
    sampleCounter = 0;
  }

  void accumulate(double error)
  {
    sampleCounter++;
    valueA += error;
    double nextErrorM = valueM + ((error - valueM) / ((double)sampleCounter));
    valueS = valueS + ((error - valueM) * (error - nextErrorM));
    valueM = nextErrorM;
  }

  double getAverage() const
  {
    return sampleCounter == 0 ? 0.0d : (valueA / ((double)sampleCounter));
  }

  double getStandardDeviation() const
  {
    return sampleCounter > 1 ? sqrt(valueS / ((double)(sampleCounter-1))) : 0.0d;
  }

  int getSampleCount() const { return sampleCounter; }

};

#endif
