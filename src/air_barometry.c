#include "air_barometry.h"
#include "math.h"

double xaGetAirMolarMass(double P, double Pv) {
    const double uv = 0.02896; // молярная масса сухого воздуха
    return uv - 0.010944 * (Pv / P);
}

double xaGetAirDensity(double T, double P, double RH) {
    double Psat = 6.1078 * pow(10, (7.5 * T - 2048.625)/(T - 35.85)); // парциальное давление насыщенного пара (в миллибарах)
    Psat *= 100; // Приводим к ПА
    double Pv = RH * Psat; // давление водяного пара
    double Pd = P - Pv; // давление сухого воздуха
    const double Rv = 461.495; // постоянная для пара (461,495 Дж/кг·К)
    const double Rd = 287.058; // газовая постоянная для сухого воздуха (287,058 Дж/кг·К)
    return (Pd / (Rd * T)) + (Pv / (Rv * T));
}

double xaGetAirPressureFromAltitude(double h, double T, double P0, double RH, double g) {
    double Psat = 6.1078 * pow(10, (7.5 * T - 2048.625)/(T - 35.85)); // парциальное давление насыщенного пара (в миллибарах)
    Psat *= 100; // Приводим к ПА
    double Pv = RH * Psat; // давление водяного пара
    const double u = xaGetAirMolarMass(P0, Pv); // молярная масса воздуха
    const double R = 8.31;  // универсальная газовая постоянная
    return P0 * exp((-u * g * h) / (R * T));
}

