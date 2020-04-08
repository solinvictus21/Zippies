
#ifndef _ZIPPYCONFIG_H_
#define _ZIPPYCONFIG_H_

#include <Arduino.h>

typedef struct _ZippyConfig
{
  double pidProportional;
  double pidIntegral;
  double pidDerivative;
  int pidSampleInterval;
  double pidOutputLimit;
  double motorDeadZone;
  double motorDeadZoneOffset;
} ZippyConfig;

void initZippyConfiguration();

int getCurrentZippyNumber();
const ZippyConfig* getCurrentZippyConfig();
int getTotalZippyCount();

extern double MOTOR_DEAD_ZONE_LEFT;
extern double MOTOR_DEAD_ZONE_RIGHT;

#endif