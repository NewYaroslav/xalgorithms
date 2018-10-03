#include <stdio.h>
#include <stdlib.h>
#include "geo_vincentys_formulae.h"
#include "geo_great_circle_distance.h"
#include "geo_geohash.h"

int main() {
    // первая точка, широта: 55 45 с.ш. долгота: 37 37 в.д.
    // вторая точка, широта: 59 53 с.ш. долгота: 30 15 в.д.
    double lat1 = xaDMStoDD('N', 55, 45, 0); double lon1 = xaDMStoDD('E', 37, 37, 0);
    double lat2 = xaDMStoDD('N', 59, 53, 0); double lon2 = xaDMStoDD('E', 30, 15, 0);
    printf("point1 %f %f\n", lat1, lon1);
    printf("point2 %f %f\n", lat2, lon2);
    return 0;
}
