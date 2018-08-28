
#ifndef _ZIPPYCOMMAND_H_
#define _ZIPPYCOMMAND_H_

#include "../lighthouse/KPath.h"
#include "../lighthouse/KVector2.h"

/*
class RelativePath// : public KPathSegment
{

private:
  KPath* path;
  double rotation;
  KVector2 offset;

public:
  RelativePath(KPath* p, double r, double offsetX, double offsetY)
    : path(p),
      rotation(r),
      offset(offsetX, offsetY)
  {}

  void lerp(double interpolatedTime, KVector2* currentTargetPosition)
  {
    path->lerp(interpolatedTime, currentTargetPosition);
    currentTargetPosition->rotate(rotation);
    currentTargetPosition->addVector(&offset);
  }

};

class LinePath// : public KPathSegment
{

private:
  KVector2 startingPosition;
  KVector2 endingPosition;

public:
  LinePath(double sx, double sy, double ex, double ey)
    : startingPosition(sx, sy),
      endingPosition(ex, ey)
  {}

  KVector2* getStartingPosition() {
    return &startingPosition;
  }

  KVector2* getEndingPosition() {
    return &endingPosition;
  }

  void lerp(double interpolatedTime, KVector2* currentTargetPosition)
  {
    double sx = startingPosition.getX();
    double sy = startingPosition.getY();
    double ex = endingPosition.getX();
    double ey = endingPosition.getY();
    currentTargetPosition->set(sx + ((ex - sx) * interpolatedTime),
        sy + ((ey - sy) * interpolatedTime));
  }

};
*/

class ZippyCommand
{

protected:
  unsigned long executionTime;

public:
  ZippyCommand()
    : executionTime(0)
  {}

  ZippyCommand(unsigned long et)
    : executionTime(et)
  {}

  unsigned long getExecutionTime() {
    // SerialUSB.println(executionTime);
    return executionTime;
  }

  void setExecutionTime(unsigned long et) {
    executionTime = et;
  }

  virtual bool getPosition(unsigned long atDeltaTime, KVector2* targetPosition) {
    return false;
  }

  virtual void getOrientation(unsigned long atDeltaTime, double* targetOrientation) {}

  virtual ~ZippyCommand() {};

};

class PathCommand : public ZippyCommand
{

private:

protected:
  KPath* path;

public:
  PathCommand(KPath* p)
    : path(p)
  {}

  PathCommand(KPath* p, unsigned long et)
    : ZippyCommand(et),
      path(p)
  {}

  bool getPosition(unsigned long atDeltaTime, KVector2* targetPosition) {
    double normalizedTime = ((double)atDeltaTime) / ((double)executionTime);
    path->lerp(normalizedTime, targetPosition);
    return true;
  }

};

#endif
