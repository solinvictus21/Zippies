
#ifndef _SCISSORPURSUITCONTROLLER_H_
#define _SCISSORPURSUITCONTROLLER_H_

#include "PursuitController.h"

class ScissorPursuitController : public PursuitController
{

public:
    ScissorPursuitController()
    {}

    void executeMove(
        const ZVector2* relativeTargetPosition,
        const ZVector2* relativeTargetVelocity);

};

#endif