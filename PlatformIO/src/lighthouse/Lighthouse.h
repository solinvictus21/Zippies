
#ifndef _LIGHTHOUSE_H_
#define _LIGHTHOUSE_H_

#include "LighthouseAxis.h"
#include "KQuaternion3.h"

typedef enum class _LighthouseState
{
  //waiting for the OOTX base station info block to download
  DownloadingBaseStationInfoBlock,
  SearchingForSignal,
  //once we have the signal locked, pause for a bit to allow the robot position to stabilize before
  //we're ready to provide position updates
  InitialPause,
  Ready
} LighthouseState;

class Lighthouse
{

private:
  LighthouseState currentState = LighthouseState::DownloadingBaseStationInfoBlock;

  LighthouseAxis axes[2];
  int currentAxis;

  KQuaternion3 orientation;
  bool receivedOrientation = false;

public:
  Lighthouse()
  {}

  void processSyncPulse(unsigned long syncPulse) {

  }

  void processSweepHitPulse(unsigned long syncHit) {

  }

};

#endif
