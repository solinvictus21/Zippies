
#ifndef _ZPATH_H_
#define _ZPATH_H_

class ZPath
{

public:
  virtual void interpolate(double interpolatedTime, KPosition* currentPosition) = 0;

  virtual ~ZPath() {}

};

#endif
