#include "geo_common.h"

double xaDegree2Rad(double degree) {
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
double xaDMStoDD(char polarity, int d, int m, double s) {
    if(polarity == 'N' || polarity == 'E') return (double)d + (double)m/60.0d + s/3600.0d;
    return -((double)d + (double)m/60.0d + s/3600.0d);
}
