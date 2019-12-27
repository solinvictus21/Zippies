
#ifndef _ZIPPYCONTROLLER_H_
#define _ZIPPYCONTROLLER_H_

class ZippyController
{

public:
  virtual void start(unsigned long startTime) {}
  virtual void loop(unsigned long currentTime) = 0;
  virtual void stop() {}

  virtual ~ZippyController() {}

};

#endif
