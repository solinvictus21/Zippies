
#ifndef _COMPOSITEPATH_H_
#define _COMPOSITEPATH_H_

#include "ZPath.h"

class CompositePath : public ZPath
{

private:
  ZPath** paths;
  double pathCount;
  double totalLength;

public:
  CompositePath(ZPath* p1, ZPath* p2) {
    paths = new ZPath*[2];
    paths[0] = p1;
    paths[1] = p2;
    pathCount = 2;
    totalLength = p1->getLength() + p2->getLength();
  }

  CompositePath(ZPath** p, int c)
    : paths(p),
      pathCount(c)
  {
    totalLength = 0.0d;
    for (int i = 0; i < pathCount; i++)
      totalLength = paths[i]->getLength();
  }

  double getLength() const { return totalLength; }

  void interpolate(
    double normalizedTime,
    KPosition* targetPosition,
    bool* reverseMotion) const
  {
    double distance = normalizedTime * totalLength;
    int currentPathIndex = 0;
    while (currentPathIndex < pathCount-1 && distance >= paths[currentPathIndex]->getLength())
      distance -= paths[currentPathIndex++]->getLength();

    //moving through first arc
    paths[currentPathIndex]->interpolate(
      distance / paths[currentPathIndex]->getLength(),
      targetPosition,
      reverseMotion);
  }

  ~CompositePath()
  {
    for (int i = 0; i < pathCount; i++)
      delete paths[i];
    delete[] paths;
  }

};

#endif
