#include "geo_vincentys_formulae.h"

double xaGetGeoDistanceUsingVincentysFormulae(double lat1, double lon1, double lat2, double lon2) {
    // константы для формы Земли
    static const double a = 6378137.0;   // meter
    static const double f = 1 / 298.257223563;
    static const double b = (1 - f) * a; // meter

    double f1 = xaDegree2Rad(lat1), l1 = xaDegree2Rad(lon1);
    double f2 = xaDegree2Rad(lat2), l2 = xaDegree2Rad(lon2);
    double L = l2 - l1;
    double tanU1 = (1-f) * tan(f1);
    double cosU1 = 1 / sqrt((1.0 + tanU1*tanU1));
    double sinU1 = tanU1 * cosU1;
    double tanU2 = (1-f) * tan(f2);
    double cosU2 = 1 / sqrt((1.0 + tanU2*tanU2));
    double sinU2 = tanU2 * cosU2;

    double sinl, cosl;
    double sinSqs, sins=0.0, coss=0.0, sig=0.0, sina, cosSqa=0, cos2sigM=0, C;
    double l = L, ll;
    int iterations = 1000;

    do {
        sinl = sin(l);
        cosl = cos(l);
        sinSqs = (cosU2*sinl) * (cosU2*sinl) + (cosU1*sinU2-sinU1*cosU2*cosl) * (cosU1*sinU2-sinU1*cosU2*cosl);
        if (sinSqs == 0)
            break; // co-incident points
        sins = sqrt(sinSqs);
        coss = sinU1*sinU2 + cosU1*cosU2*cosl;
        sig = atan2(sins, coss);
        sina = cosU1 * cosU2 * sinl / sins;
        cosSqa = 1 - sina*sina;
        cos2sigM = (cosSqa != 0) ? (coss - 2*sinU1*sinU2/cosSqa) : 0; // equatorial line: cosSqα=0 (§6)
        C = f/16*cosSqa*(4+f*(4-3*cosSqa));
        ll = l;
        l = L + (1-C) * f * sina * (sig + C*sins*(cos2sigM+C*coss*(-1+2*cos2sigM*cos2sigM)));
        if (abs(l) > M_PI)
            return 0.0;
    } while (abs(l-ll) > 1e-12 && iterations--);

    if (!iterations)
        return 0.0;

    double uSq = cosSqa * (a*a - b*b) / (b*b);
    double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
    double dsig = B*sins*(cos2sigM+B/4*(coss*(-1+2*cos2sigM*cos2sigM)-
                                  B/6*cos2sigM*(-3+4*sins*sins)*(-3+4*cos2sigM*cos2sigM)));
    double s = b*A*(sig-dsig);
    return s;
}
