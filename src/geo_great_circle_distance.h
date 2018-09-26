#ifndef GREAT_CIRCLE_DISTANCE_HPP_INCLUDED
#define GREAT_CIRCLE_DISTANCE_HPP_INCLUDED

#include "geo_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define XA_EARTH_RADIUS (6371.008 * 1000.0)     /// радиус Земли в метрах
#define XA_ACTUAL_GRAVITY 9.81665               /// ускорение свободного падения по умолчанию

/** \brief Получить расстояние между двумя точками географического местоположения используя расстояние по большому кругу
 * Кратчайшее расстояние между двумя точками на поверхности сферы , измеренное вдоль поверхности сферы.
 * В расчете берется среднее значение радиуса Земли (6371.008 км), что должно давать ошибку около 0.5%
 * \param lat1 широта первой точки
 * \param lon1 долгота первой точки
 * \param lat2 широта второй точки
 * \param lon2 долгота второй точки
 * \return расстояние между точками в метрах
 */
double xaGetGeoDistanceUsingGreatCircleDistance(double lat1, double lon1, double lat2, double lon2);

#ifdef __cplusplus
}
#endif

#endif // GREAT_CIRCLE_DISTANCE_HPP_INCLUDED
