
#ifndef _PATHDATA_H_
#define _PATHDATA_H_

#include "lighthouse/KVector2.h"

// cat points_from_vrep_export.csv | wc -l
// #define PATH_POINT_COUNT        69
#define PATH_POINT_COUNT        19
// cat points_from_vrep_export.csv | cut -d, -f1,2 | sed "s/\(.*\)/{ \1 },/g"
extern KVector2 PATH_POINTS[PATH_POINT_COUNT];

#endif
