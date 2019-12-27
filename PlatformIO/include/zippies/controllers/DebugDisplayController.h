
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

  unsigned long previousDisplayTime = 0;
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

  void start(unsigned long currentTime);
  void loop(unsigned long currentTime);
  void stop();


};

#endif
