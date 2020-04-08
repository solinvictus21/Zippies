
#ifndef _PID_V2_h_
#define _PID_V2_h_

/**
 * This PID controller was adapted from the Arduino PID_v1 open source library to be simpler to use while
 * addressing some concerns over the previous mechanism of using sample rate timing to calculate each
 * update. Instead, a "samples per second" is specified and the timing of each update is managed externally.
 * In the case of a Zippy controlled by the Lighthouse poosition tracking, updates occur each time sweep
 * cycles have been detected in both the X and Z axes, which is guaranteed to be 60 times each second, so
 * this change allows the PID to be time-locked exactly to the Lighthouse inputs.
 */
class PID_v2
{

private:
  int samplesPerSecond;
  double setPoint;
  double kp, ki, kd;
  double outMin, outMax;
  bool pOnE;
  bool isReversed;

  bool isStarted = false;
  double previousInput = 0.0d;
	double outputSum = 0.0d;

public:
  PID_v2(
    int samplesPerSecond, double setPoint,
    double kP, double kI, double kD,
    double outMin, double outMax,
    bool pOnE, bool reverseMode);

  void setTunings(double, double, double);
  void setOutputLimits(double, double);
  void setReverseMode(bool);
  bool isReverseMode() const { return isReversed; }

  void start();
  double compute(double input);
  void stop();

};

#endif
