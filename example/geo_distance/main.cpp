#include <iostream>
#include "geo_vincentys_formulae.h"
#include "geo_great_circle_distance.h"
#include "geo_geohash.h"

using namespace std;

int main() {

    // первая точка, широта: 55 45 с.ш. долгота: 37 37 в.д.
    // вторая точка, широта: 59 53 с.ш. долгота: 30 15 в.д.
    double lat1 = 55.0d + 45.0d/60.0d; double lon1 = 37.0d + 37.0d/60.0d;
    double lat2 = 59.0d + 53.0d/60.0d; double lon2 = 30.0d + 15.0d/60.0d;

    std::cout << "lat1 " << lat1 << " lon1 " << lon1 << std::endl;
    std::cout << "lat2 " << lat2 << " lon2 " << lon2 << std::endl;

    std::cout << "great circle distance: " << xaGetGeoDistanceUsingGreatCircleDistance(lat1, lon1, lat2, lon2) << " m" << std::endl; // расстояние: 633007 м
    std::cout << "vincentys formulae: " << xaGetGeoDistanceUsingVincentysFormulae(lat1, lon1, lat2, lon2) << " m" << std::endl;

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
