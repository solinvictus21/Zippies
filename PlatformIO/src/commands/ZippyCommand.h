
#ifndef _ZIPPYCOMMAND_H_
#define _ZIPPYCOMMAND_H_

class ZippyCommand
{

public:
  virtual void start(unsigned long currentTime) = 0;
  virtual bool loop(unsigned long currentTime) = 0;
  virtual ~ZippyCommand() {};

};

#endif
