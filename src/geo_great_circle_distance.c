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

void xaGetGeoLocationUsingGreatCircleDistance(double lat1, double lon1, double x2, double y2, double* lat2, double* lon2) {
    const double PI = 3.1415926535897932384626433832795d;
    double lat1Rad = xaDegree2Rad(lat1);
    double deltaLat = x2 / XA_EARTH_RADIUS;
    double deltaLon = y2 / (XA_EARTH_RADIUS * cos(lat1Rad));
    *lat2 = (lat1Rad + deltaLat) * 180.0 / PI;
    *lon2 = (xaDegree2Rad(lon1) + deltaLon) * 180.0 / PI;
}

void xaGetPointUsingGreatCircleDistance(double lat1, double lon1,double lat2, double lon2, double* x2, double* y2) {
    const double PI = 3.1415926535897932384626433832795d;
    double lat1Rad = xaDegree2Rad(lat1);
    *y2 = (XA_EARTH_RADIUS * cos(lat1Rad)) * ((lon2 * PI/ 180.0) - xaDegree2Rad(lon1));
    *x2 = (((lat2 * PI) / 180.0) - lat1Rad) * XA_EARTH_RADIUS;
}
