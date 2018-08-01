
#ifndef _AUTODRIVEDATA_H_
#define _AUTODRIVEDATA_H_


// cat points_from_vrep_export.csv | wc -l
// #define PATH_POINT_COUNT       70
// #define PATH_POINT_COUNT       150
#define PATH_POINT_COUNT       230
// cat points_from_vrep_export.csv | cut -d, -f1,2 | sed "s/\(.*\)/{ \1 },/g"
extern double PATH_POINTS[PATH_POINT_COUNT][2];

#endif
