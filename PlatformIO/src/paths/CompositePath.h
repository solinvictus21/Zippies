
#ifndef _COMPOSITEPATH_H_
#define _COMPOSITEPATH_H_

#include "ZPath.h"

class CompositePath : public ZPath
{

private:
  const ZPath** paths;
  int pathCount;
  double totalLength;

public:
  CompositePath(const ZPath* p1, const ZPath* p2) {
    paths = new const ZPath*[2];
    paths[0] = p1;
    paths[1] = p2;
    pathCount = 2;
    totalLength = p1->getLength() + p2->getLength();
  }

  CompositePath(const ZPath** p, int c)
    : paths(p),
      pathCount(c)
  {
    totalLength = 0.0d;
    for (int i = 0; i < pathCount; i++)
      totalLength = paths[i]->getLength();
  }

  bool updatesPosition() const { return true; }
  double getLength() const { return totalLength; }

  void interpolate(
    double normalizedTime,
    KMatrix2* targetPosition) const
  {
    double distance = normalizedTime * totalLength;
    int currentPathIndex = 0;
    while (currentPathIndex < pathCount-1 && distance >= paths[currentPathIndex]->getLength())
      distance -= paths[currentPathIndex++]->getLength();

    //moving through first arc
    paths[currentPathIndex]->interpolate(
      distance / paths[currentPathIndex]->getLength(),
      targetPosition);
  }

  ~CompositePath()
  {
    for (int i = 0; i < pathCount; i++)
      delete paths[i];
    delete[] paths;
  }

};

#endif
