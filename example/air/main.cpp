#include <iostream>
#include "air_barometry.h"
#include "geo_common.h"
#include "geo_acceleration_gravity.h"
#include "air_damping.h"

using namespace std;

int main() {
    const double Tc = 30.0d; // температура воздуха в цельсиях
    const double P0 = XA_TECHNICAL_ATMOSPHERE; // давление воздуха
    const double RH = 20.0d;    // относительная влажность
    const double h = 0.0d;      // высота над уровнем моря
    double lat = xaDMStoDD(true, 55, 45, 0); // широта
    double g = xaGetAccelerationGravityLatitude(lat); // ускорение свободного падения
    double vel = 10.0; // скорость тела
    double vel_wind = -2.0; // скорость ветра в обратную сторону
    double Cxo = 0.47; // 0.47 это безразмерный аэродинамический коэффициент сопротивления для сферы
    double r = 0.5; // радиус сферического тела
    double S = r * r * 3.1415d; // площать сечения сферы
    double p = xaGetAirDensity(-XA_ABSOLUTE_ZERO_TEMP + Tc, P0, RH);
    cout << "air density: " << p << endl;
    cout << "air pressure: " << xaGetAirPressureFromAltitude(h, -XA_ABSOLUTE_ZERO_TEMP + Tc, P0, RH, g) << endl;
    cout << "air resistance force: " << xaGetAirResistanceForce(vel, vel_wind, Cxo, S, p) << endl;
    return 0;
}
