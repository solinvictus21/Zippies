
#ifndef _ZIPPYCONTROLLER_H_
#define _ZIPPYCONTROLLER_H_

class ZippyController
{

public:
  virtual void start() {}
  virtual void loop(unsigned long deltaTime) = 0;
  virtual void stop() {}

  virtual ~ZippyController() {}

};

#endif
