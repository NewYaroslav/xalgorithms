#ifndef GREAT_CIRCLE_DISTANCE_HPP_INCLUDED
#define GREAT_CIRCLE_DISTANCE_HPP_INCLUDED

#include "geo_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define XA_EARTH_RADIUS (6371.008 * 1000.0)     /// радиус Земли в метрах

/** \brief Получить расстояние между двумя точками географического местоположения используя расстояние по большому кругу
 * Кратчайшее расстояние между двумя точками на поверхности сферы , измеренное вдоль поверхности сферы.
 * В расчете берется среднее значение радиуса Земли (6371.008 км), что должно давать ошибку около 0.5%
 * Формула Хаверсина (от RW Sinnott, «Достоинства Хаверсине», «Небо и телескоп», том 68, № 2, 1984, стр. 159)
 * https://www.movable-type.co.uk/scripts/gis-faq-5.1.html
 * \param lat1 широта первой точки
 * \param lon1 долгота первой точки
 * \param lat2 широта второй точки
 * \param lon2 долгота второй точки
 * \return расстояние между точками в метрах
 */
double xaGetGeoDistanceUsingGreatCircleDistance(double lat1, double lon1, double lat2, double lon2);

/** \brief Получить
 * X - широта (север положителен), вверх
 * Y - долгота (восток положителен), направо
 * \param lat1 широта первой точки
 * \param lon1 долгота первой точки
 * \param x2 направление вдоль широты (смещение в метра от первой точки)
 * \param y2 направление вдоль долготы (смещение в метра от первой точки)
 * \param lat2 широта второй точки
 * \param lon2 долгота второй точки
 */
void xaGetGeoLocationUsingGreatCircleDistance(double lat1, double lon1, double x2, double y2, double* lat2, double* lon2);


#ifdef __cplusplus
}
#endif

#endif // GREAT_CIRCLE_DISTANCE_HPP_INCLUDED
