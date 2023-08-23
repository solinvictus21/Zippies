
#ifndef _STANLEYPURSUITCONTROLLER_H_
#define _STANLEYPURSUITCONTROLLER_H_

#include "PursuitController.h"

class StanleyPursuitController : public PursuitController
{

public:
    StanleyPursuitController()
    {}

    void executeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);

};

#endif