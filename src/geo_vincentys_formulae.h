#ifndef VINCENTYS_FORMULAE_HPP_INCLUDED
#define VINCENTYS_FORMULAE_HPP_INCLUDED

#include "geo_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Получить расстояние между двумя точками географического местоположения используя формулу Винченти
 * Формула Винченти дает высокую точность. Однако, если точки почти антиподальны, алгоритм не сходится.
 * \param lat1 широта первой точки
 * \param lon1 долгота первой точки
 * \param lat2 широта второй точки
 * \param lon2 долгота второй точки
 * \return расстояние между точками в метрах
 */
double xaGetGeoDistanceUsingVincentysFormulae(double lat1, double lon1, double lat2, double lon2);

#ifdef __cplusplus
}
#endif

#endif // VINCENTYS_FORMULAE_HPP_INCLUDED
