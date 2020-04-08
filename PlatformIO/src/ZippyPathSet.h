
#ifndef _ZIPPYPATHSET_H_
#define _ZIPPYPATHSET_H_

#include "zippies/ZippyRoutine.h"

#define FOLLOW_RADIUS_LARGE 100.0d
#define FOLLOW_RADIUS_SMALL  50.0d

extern PathDefinition synchronizedPath[];
extern const int synchronizedPathLength;

extern PathDefinition followPath[];
extern const int followPathLength;

#endif
