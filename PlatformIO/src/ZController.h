
#ifndef _ZCONTROLLER_H_
#define _ZCONTROLLER_H_

class ZController
{

public:
  virtual void start(unsigned long startTime) {}
  virtual void loop(unsigned long currentTime) = 0;
  virtual void stop() {}

  virtual ~ZController() {}
  
};

#endif
