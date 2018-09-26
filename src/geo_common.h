#ifndef GEO_COMMON_HPP_INCLUDED
#define GEO_COMMON_HPP_INCLUDED

#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

inline double xaDegree2Rad(double degree) {
    static const double coef = M_PI/180.0;
    return degree*coef;
}

#ifdef __cplusplus
}
#endif

#endif // GEO_сOMMON_HPP_INCLUDED
