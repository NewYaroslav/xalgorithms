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

/** \brief Преобразовать градусы, минуты и секунды в десятичные градусы
 * \param polarity - флаг, установить в true для северной широты (N) или для восточного полушария (E)
 * \param d - градусы
 * \param m - минуты
 * \param s - секунды
 * \return десятичные градусы
 *
 */
inline double xaDMStoDD(bool polarity, int d, int m, double s) {
    return polarity ? (double)d + (double)m/60.0d + s/3600.0d : -((double)d + (double)m/60.0d + s/3600.0d);
}

#ifdef __cplusplus
}
#endif

#endif // GEO_сOMMON_HPP_INCLUDED
