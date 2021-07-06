
#ifndef _PURSUITCONTROLLER_H_
#define _PURSUITCONTROLLER_H_

#include "zippies/ZippyMath.h"

#define LINEAR_EPSILON                               10.00  //1.0cm
// #define LINEAR_EPSILON                               20.00  //2cm
// #define LINEAR_EPSILON                               30.00  //3cm

#define ANGULAR_EPSILON                               0.052359877559830  //3 degrees
// #define ANGULAR_EPSILON                               0.069813170079773  //4 degrees
// #define ANGULAR_EPSILON                               0.087266462599716  //5 degrees
// #define ANGULAR_EPSILON                               0.104719755119660  //6 degrees
// #define ANGULAR_EPSILON                               0.139626340159546  //8 degrees
// #define ANGULAR_EPSILON                               0.174532925199433  //10 degrees
// #define ANGULAR_EPSILON                               0.261799387799149  //15 degrees

// #define MAX_STATE_DOWNGRADE_ITERATIONS               30
#define MAX_STATE_DOWNGRADE_ITERATIONS               60

typedef enum class _MovementState
{
  Stopped,
  Turning,
  Moving,
} MovementState;

class PursuitController
{

public:
    PursuitController() {}

    virtual void continuePursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity) = 0;

    virtual void stopPursuit(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity) = 0;

    // virtual void stopPursuit(const ZMatrix2* relativeTargetPosition) {}

    virtual void stop() = 0;

    virtual bool isStopped() = 0;

    virtual ~PursuitController() {};

};

#endif