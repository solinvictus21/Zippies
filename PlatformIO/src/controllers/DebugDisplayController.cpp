
#include "zippies/controllers/DebugDisplayController.h"

#define DEBUG_DISPLAY_INTERVAL        1000

void DebugDisplayController::start(unsigned long currentTime)
{
  face.clearScreen();
  previousDisplayTime = currentTime;
}

void DebugDisplayController::loop(unsigned long currentTime)
{
  //capture sensor #0 stats
//   const LighthouseSensorHitCycle* cycle = sensors->getLeftSensor()->getXHitCycle();
//   SerialUSB.println(cycle->syncTicks + cycle->sweepHitStartTicks);
  captureAxisStatistics(sensors->getLeftSensor()->getXHitCycle(), &xSyncAccumulator, &xSweepHitStartAccumulator, &xSweepHitEndAccumulator);
  captureAxisStatistics(sensors->getLeftSensor()->getYHitCycle(), &ySyncAccumulator, &ySweepHitStartAccumulator, &ySweepHitEndAccumulator);

  if (currentTime - previousDisplayTime < DEBUG_DISPLAY_INTERVAL)
    return;
  previousDisplayTime += DEBUG_DISPLAY_INTERVAL;

  face.clearScreen();

  /*
  //position data on the left
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t fontHeight = face.getFontHeight();
  const ZMatrix2* currentPosition = sensors->getPosition();
  face.displayLabelAndData(x, y, "X", currentPosition->position.getX());
  y += fontHeight;
  face.displayLabelAndData(x, y, "Y", currentPosition->position.getY());
  y += fontHeight;
  face.displayLabelAndData(x, y, "O", (180.0d * currentPosition->orientation.get()) / M_PI);
  */

  //display sync pulse stats
  uint8_t y = 0;
  uint8_t fontHeight = face.getFontHeight();
  face.displayLabelAndData(
      0, y,
      "XS", xSyncAccumulator.getAverage());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "XC", xSyncAccumulator.getSampleCount());
  //display x axis stats
  y += fontHeight;
  face.displayLabelAndData(
      0, y,
      "V", xSweepHitStartAccumulator.getAverage(), 0);
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "D", xSweepHitStartAccumulator.getStandardDeviation());
  y += fontHeight;
  face.displayLabelAndData(
      0, y,
      "W", xSweepHitEndAccumulator.getAverage());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "D", xSweepHitEndAccumulator.getStandardDeviation());
  /*
  face.displayLabelAndData(
      0, y,
      "C", xSweepHitStartAccumulator.getSampleCount());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "+", xSyncAccumulator.getSampleCount() - xSweepHitStartAccumulator.getSampleCount());
  // */

  //display y axis stats
  y += fontHeight;
  face.displayLabelAndData(
      0, y,
      "YS", ySyncAccumulator.getAverage());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "YC", ySyncAccumulator.getSampleCount());
  y += fontHeight;
  face.displayLabelAndData(
      0, y,
      "V", ySweepHitStartAccumulator.getAverage(), 0);
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "D", ySweepHitStartAccumulator.getStandardDeviation());
  y += fontHeight;
  face.displayLabelAndData(
      0, y,
      "W", ySweepHitEndAccumulator.getAverage());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "D", ySweepHitEndAccumulator.getStandardDeviation());
  /*
  face.displayLabelAndData(
      0, y,
      "C", ySweepHitStartAccumulator.getSampleCount());
  face.displayLabelAndData(
      SCREEN_WIDTH_PIXELS_2, y,
      "+", ySyncAccumulator.getSampleCount() - xSweepHitStartAccumulator.getSampleCount());
  // */

  xSyncAccumulator.reset();
  xSweepHitStartAccumulator.reset();
  xSweepHitEndAccumulator.reset();
  ySyncAccumulator.reset();
  ySweepHitStartAccumulator.reset();
  ySweepHitEndAccumulator.reset();
}

void DebugDisplayController::captureAxisStatistics(
    const LighthouseSensorHitCycle* hitCycle,
    StatisticsAccumulator* syncAccumulator,
    StatisticsAccumulator* sweepHitStartAccumulator,
    StatisticsAccumulator* sweepHitEndAccumulator)
{
  syncAccumulator->accumulate((hitCycle->syncTicks - SYNC_PULSE_MIN) % SYNC_PULSE_AXIS_WINDOW);
  if (hitCycle->sweepHitStartTicks) {
    sweepHitStartAccumulator->accumulate(hitCycle->syncTicks + hitCycle->sweepHitStartTicks);
    if (hitCycle->sweepHitEndTicks)
      sweepHitEndAccumulator->accumulate(hitCycle->sweepHitEndTicks);
  }
}

void DebugDisplayController::stop()
{
  face.displayText(0, 6*face.getFontHeight(), "Offline");
}
