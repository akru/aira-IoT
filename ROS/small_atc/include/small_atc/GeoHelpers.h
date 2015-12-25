#ifndef __GEO_HELPERS_H__
#define __GEO_HELPERS_H__

#include "DynamicMap.h" 

#include <cmath>
#include <dron_common_msgs/SatFix.h>
#include <geometry_msgs/Point.h>

using namespace geometry_msgs;
using namespace dron_common_msgs;

/**
 * Global position system coords to length converter
 **/
#define RADIUS_OF_EARTH 6371 

inline double grad2len(double grad)
{ return M_PI * RADIUS_OF_EARTH * grad / 180; }

inline double len2grad(double len)
{ return len * 180 / M_PI / RADIUS_OF_EARTH; }

inline Point satFix2Point(const SatFix &fix, const MapMetaData &meta) {
    Point pt;
    pt.x = grad2len(meta.origin.latitude - fix.latitude) / meta.resolution;
    pt.y = grad2len(meta.origin.longitude - fix.longitude) / meta.resolution;
    pt.z = (meta.origin.altitude - fix.altitude) / meta.resolution;
    return pt;
}

inline SatFix point2SatFix(const Point &pt, const MapMetaData &meta) {
    SatFix fix;
    fix.latitude = meta.origin.latitude + len2grad(pt.x * meta.resolution);
    fix.longitude = meta.origin.longitude + len2grad(pt.y * meta.resolution);
    fix.latitude = meta.origin.altitude + pt.z * meta.resolution;
    return fix;
}

#endif
