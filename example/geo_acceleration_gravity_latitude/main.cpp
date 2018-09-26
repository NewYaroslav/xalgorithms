#include <iostream>
#include "geo_common.h"
#include "geo_acceleration_gravity.h"

using namespace std;

int main()
{
    cout << "lat = " << xaDMStoDD(true, 55, 45, 0) << " g = " << xaGetAccelerationGravityLatitude(xaDMStoDD(true, 55, 45, 0)) << " m/s^2" << endl;
    return 0;
}
