#include "air_damping.h"

double xaGetAirResistanceForce(double vel, double vel_wind, double Cxo, double S, double p) {
    double dvel = vel - vel_wind;
    if(abs(dvel) > 0.0) {
        double force = dvel * dvel * Cxo * p * S / 2.0d;
        if(dvel > 0) force = -force;
        return force;
    }
    return 0.0d;
}
