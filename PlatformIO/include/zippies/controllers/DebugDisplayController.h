
#ifndef _DEBUGDISPLAYCONTROLLER_H_
#define _DEBUGDISPLAYCONTROLLER_H_

#include "ZippyController.h"
#include "zippies/hardware/SensorFusor.h"
#include "zippies/hardware/ZippyFace.h"
#include "zippies/math/StatisticsAccumulator.h"

class DebugDisplayController : public ZippyController
{

private:
  SensorFusor* sensors;
  StatisticsAccumulator xSyncAccumulator;
  StatisticsAccumulator xSweepHitStartAccumulator;
  StatisticsAccumulator xSweepHitEndAccumulator;
  StatisticsAccumulator ySyncAccumulator;
  StatisticsAccumulator ySweepHitStartAccumulator;
  StatisticsAccumulator ySweepHitEndAccumulator;

  unsigned long timeSinceLastDisplay = 0;
  ZippyFace face;

  void captureAxisStatistics(
      const LighthouseSensorHitCycle* hitCycle,
      StatisticsAccumulator* syncAccumulator,
      StatisticsAccumulator* sweepHitStartAccumulator,
      StatisticsAccumulator* sweepHitEndAccumulator);

public:
  DebugDisplayController(SensorFusor* s)
    : sensors(s)
  {}

  void start();
  bool loop(unsigned long deltaTime);
  void stop();


};

#endif
