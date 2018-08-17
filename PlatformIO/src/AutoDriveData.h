
#ifndef _AUTODRIVEDATA_H_
#define _AUTODRIVEDATA_H_

#include "lighthouse/KVector.h"

// cat points_from_vrep_export.csv | wc -l
/* vectors
#define PATH_POINT_COUNT       203
*/
// /* beziers
#define PATH_POINT_COUNT        69
// */
// cat points_from_vrep_export.csv | cut -d, -f1,2 | sed "s/\(.*\)/{ \1 },/g"
// extern double PATH_POINTS[PATH_POINT_COUNT][2];
extern KVector2 PATH_POINTS[PATH_POINT_COUNT];

#endif
