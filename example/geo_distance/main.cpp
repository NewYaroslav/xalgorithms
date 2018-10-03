#include <iostream>
#include "geo_vincentys_formulae.h"
#include "geo_great_circle_distance.h"
#include "geo_geohash.h"

using namespace std;

int main() {

    // первая точка, широта: 55 45 с.ш. долгота: 37 37 в.д.
    // вторая точка, широта: 59 53 с.ш. долгота: 30 15 в.д.
    double lat1 = xaDMStoDD(true, 55, 45, 0); double lon1 = xaDMStoDD(true, 37, 37, 0);
    double lat2 = xaDMStoDD(true, 59, 53, 0); double lon2 = xaDMStoDD(true, 30, 15, 0);

    std::cout << "lat1 " << lat1 << " lon1 " << lon1 << std::endl;
    std::cout << "lat2 " << lat2 << " lon2 " << lon2 << std::endl;

    std::cout << "great circle distance: " << xaGetGeoDistanceUsingGreatCircleDistance(lat1, lon1, lat2, lon2) << " m" << std::endl; // расстояние: 633007 м
    std::cout << "vincentys formulae: " << xaGetGeoDistanceUsingVincentysFormulae(lat1, lon1, lat2, lon2) << " m" << std::endl;

    double new_lat;
    double new_lon;
    double new_x = 500;
    double new_y = 500;
    xaGetGeoLocationUsingGreatCircleDistance(lat1, lon1, new_x, new_y, &new_lat, &new_lon);
    std::cout << "new lat lon: " << new_lat << " " << new_lon << std::endl;
    double new_dist = sqrt(new_x*new_x + new_y*new_y);
    std::cout << "new distance: " << new_dist << " m" << std::endl;
    std::cout << "new great circle distance: " << xaGetGeoDistanceUsingGreatCircleDistance(lat1, lon1, new_lat, new_lon) << " m" << std::endl;


    unsigned long long geohash_prec8 = xaGeohashEncode(lat1, lon1, 8);
    unsigned long long geohash_prec9 = xaGeohashEncode(lat1, lon1, 9);
    unsigned long long geohash_prec12 = xaGeohashEncode(lat1, lon1, 12);

    std::cout << "geohash prec 8: " << std::endl;
    for(int i = 63; i >= 0; --i) {
		printf("%d", (geohash_prec8 >> i) & 1);
	}
	printf("\n");
    std::cout << "geohash prec 9: " << std::endl;
	for(int i = 63; i >= 0; --i) {
		printf("%d", (geohash_prec9 >> i) & 1);
	}
	printf("\n");
    std::cout << "geohash prec 12: " << std::endl;
	for(int i = 63; i >= 0; --i) {
		printf("%d", (geohash_prec12 >> i) & 1);
	}
	printf("\n");

    return 0;
}
