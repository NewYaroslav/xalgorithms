#ifndef GEO_ACCELERATION_GRAVITY_H_INCLUDED
#define GEO_ACCELERATION_GRAVITY_H_INCLUDED

#include "math.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Получить ускорение свободного падения в зависимости от широты
 * \param lat широта
 * \param h - высота над уровнем моря
 * \return ускорение свободного падения
 */
inline double xaGetAccelerationGravityLatitude(double lat, double h = 0.0d) {
    const double PI = 3.1415926535897932384626433832795d;
    double rad = lat * PI / 180.0d;
    double coeff1 = sin(rad);
    coeff1 = coeff1 * coeff1;
    double coeff2 = sin(2 * rad);
    coeff2 = coeff2 * coeff2;
    return 9.780318d * (1.0d + 0.005302d * coeff1 - 0.000006d * coeff2) - 0.000003086d * h;
}

#ifdef __cplusplus
}
#endif

#endif // GEO_ACCELERATION_GRAVITY_H_INCLUDED
