#include "geo_great_circle_distance.h"

double xaGetGeoDistanceUsingGreatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
    double deltaLon = xaDegree2Rad(lon2 - lon1);
    double deltaLat = xaDegree2Rad(lat2 - lat1);
    double a = pow(sin(deltaLat / 2.0d), 2.0d) +
        cos(xaDegree2Rad(lat1))*
        cos(xaDegree2Rad(lat2))*
        pow(sin(deltaLon / 2.0d), 2.0d);
    double c = 2.0d * atan2(sqrt(a), sqrt(1.0d - a));
    return XA_EARTH_RADIUS * c;
}
