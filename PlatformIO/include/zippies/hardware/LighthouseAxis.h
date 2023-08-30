
#ifndef _LIGHTHOUSEAXIS_H_
#define _LIGHTHOUSEAXIS_H_

class LighthouseAxis
{

private:
  //positive phase indicates that the beam is ahead of the ideal, which means that it will strike the sensors before we think it
  //should, leading us to believe that the angle is smaller than it is; thus we must add the phase for each rotor to the angle to
  double phase = 0.0d;
  double curve = 0.0d;
  double tilt = 0.0d;
  //gibbous magnitude indicates how much faster or slower the beam moves through the cycle than we expect; gibbous phase indicates
  //the center point in radians around which the magnitude scales the speed of the beam; so when gibbous phase is zero, the gibbous
  //magnitude scales the speed of the beam around the ideal center point and enters and leaves the field of view at equal offsets
  //at both the start and end of the cycle; positive gibbous phase indicates that the laser matches the ideal after the center
  double gibbousPhase = 0.0d;
  double gibbousMagnitude = 0.0d;

  //number of ticks from the end of the sync signal to the start of the sweep hit
  unsigned long pendingSyncTicks = 0;
  unsigned long pendingSweepStartTicks = 0;
  unsigned long pendingSweepHitTicks = 0;
  unsigned long pendingSweepEndTicks = 0;

  //total number of ticks from the start of the sync signal to the start of the sweep hit
  unsigned long sweepHitTicks = 0;
  //updated each time a sweep hit is detected; set to zero when we fail to detect the lighthouse sweep
  unsigned long sweepHitTimeStamp = 0;

public:
  LighthouseAxis()
  {}


};

#endif
