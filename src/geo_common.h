#ifndef GEO_COMMON_HPP_INCLUDED
#define GEO_COMMON_HPP_INCLUDED

#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern inline double xaDegree2Rad(double degree);

/** \brief Преобразовать градусы, минуты и секунды в десятичные градусы
 * \param polarity флаг, для положительных значений установить 'N' для северной широты или для восточного полушария (E)
 * \param d градусы
 * \param m минуты
 * \param s секунды
 * \return десятичные градусы
 */
extern inline double xaDMStoDD(char polarity, int d, int m, double s);

#ifdef __cplusplus
}
#endif

#endif // GEO_сOMMON_HPP_INCLUDED
