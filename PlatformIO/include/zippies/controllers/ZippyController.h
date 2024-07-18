
#ifndef _ZIPPYCONTROLLER_H_
#define _ZIPPYCONTROLLER_H_

/**
 * Starting a new evolution of the controller hierarchy which relies on accuracy in microseconds
 * and provides both a "current time" that is zero-based from the moment start() is invoked.
 * 
 * This eliminates the need for individual controllers to maintain an elapsed time or having to
 * individually calculate the delta time.
 */
class ZippyController
{

public:

    //when start is invoked, the relative time is zero
    virtual bool start() = 0;
    virtual bool loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros) = 0;
    virtual void stop() {}

    virtual ~ZippyController() {}

};

#endif
