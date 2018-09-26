#ifndef GEOHASH_HPP_INCLUDED
#define GEOHASH_HPP_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

static const int XA_GEOHASH_MAX_PRECISION = 12; /**< Максимальная возможная длина геохэш-строки 12 символов */

inline unsigned long long xaInterleave(unsigned long long x, unsigned long long y) {
    x = (x | (x << 16)) & 0x0000ffff0000ffff;
    x = (x | (x << 8)) & 0x00ff00ff00ff00ff;
    x = (x | (x << 4)) & 0x0f0f0f0f0f0f0f0f;
    x = (x | (x << 2)) & 0x3333333333333333;
    x = (x | (x << 1)) & 0x5555555555555555;

    y = (y | (y << 16)) & 0x0000ffff0000ffff;
    y = (y | (y << 8)) & 0x00ff00ff00ff00ff;
    y = (y | (y << 4)) & 0x0f0f0f0f0f0f0f0f;
    y = (y | (y << 2)) & 0x3333333333333333;
    y = (y | (y << 1)) & 0x5555555555555555;

    return x | (y << 1);
}

/** \brief Кодирует географическое местоположение
 * Geohash - система геокодирования общего пользования
 * Geohash предлагает такие свойства, как произвольная точность и возможность постепенного удаления символов с конца кода,
 * чтобы уменьшить его размер (и постепенно потерять точность). Вследствие постепенной деградации точности соседние места часто
 * (но не всегда) имеют аналогичные префиксы. Чем дольше общий префикс, тем ближе эти два места.
 * Максимальная возможная длина геохэш-строки 12 символов
 * \param lat долгота
 * \param lon широта
 * \param prec точность (от 1 до 12)
 * \return вернет geohash
 */
inline unsigned long long xaGeohashEncode(double lat, double lon, int prec) {
    lat = lat/180.0 + 1.5;
    lon = lon/360.0 + 1.5;
    unsigned long long ilat = *((unsigned long long*)&lat);
    unsigned long long ilon = *((unsigned long long*)&lon);
    ilat >>= 20;
    ilon >>= 20;
    ilat &= 0x00000000ffffffff;
    ilon &= 0x00000000ffffffff;
    return xaInterleave(ilat, ilon) >> (XA_GEOHASH_MAX_PRECISION-prec)*5;
}

/** \brief Сравнить две точки географического местоположения используя Geohash
 * \param lat1 долгота первой точки
 * \param lon1 широта первой точки
 * \param lat2 долгота второй точки
 * \param lon2 широта второй точки
 * \param precision точность сравнения (от 0 до 12)
 * \return вернет 0 в случае совпадения точек
 */
inline int xaGeohashComparePoints(double lon1, double lat1,
                            double lon2, double lat2,
                            int precision) {
    if(precision >= 1 && precision <= XA_GEOHASH_MAX_PRECISION) {
        unsigned long long gh1 = xaGeohashEncode(lat1, lon1, precision);
        unsigned long long gh2 = xaGeohashEncode(lat2, lon2, precision);
        return gh1 - gh2;
    }
    return 0xffffffff;
}

#ifdef __cplusplus
}
#endif

#endif // GEOHASH_HPP_INCLUDED
