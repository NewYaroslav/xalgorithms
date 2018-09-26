## xalgorithms
Сборник различных алгоритмов на C и C++

## Как установить\использовать
Код представлен в виде *.h .hpp* и *.c .cpp* файлов. Добавьте файлы в свой проект.

Все функции сборника алгоритмов начинаются с *xa*, чтобы избежать конфликтов со сторонними библиотеками.

## Алгоритмы для C и C++
+ Работа с геоданными

Примеры: *example\geo_distance*,*example\geo_acceleration_gravity_latitude*

**xaGetGeoDistanceUsingVincentysFormulae** - расстояние между двумя точками географического местоположения используя формулу Винченти

**xaGetGeoDistanceUsingGreatCircleDistance** - расстояние между двумя точками географического местоположения используя расстояние по большому кругу

**xaGeohashEncode** - Кодирует географическое местоположение в Geohash

**xaGeohashComparePoints** - Сравнить две точки географического местоположения используя Geohash

**xaDMStoDD** - Преобразовать градусы, минуты и секунды в десятичные градусы

**xaGetAccelerationGravityLatitude** - Получить ускорение свободного падения в зависимости от широты

+ Барометрия

**xaGetAirDensity** - Получить плотность воздуха

**xaGetAirPressureFromAltitude** - Получить давление на высоте над уровнем моря

Примеры: *example\air*

 ```
#include <iostream>
#include "air_barometry.h"
#include "geo_common.h"
#include "geo_acceleration_gravity.h"

using namespace std;

int main() {
    const double Tc = 30.0d; // температура воздуха в цельсиях
    const double P0 = XA_TECHNICAL_ATMOSPHERE; // давление воздуха
    const double RH = 20.0d;    // относительная влажность
    const double h = 0.0d;      // высота над уровнем моря
    double lat = xaDMStoDD(true, 55, 45, 0); // широта
    double g = xaGetAccelerationGravityLatitude(lat); // ускорение свободного падения
    cout << "air density: " << xaGetAirDensity(-XA_ABSOLUTE_ZERO_TEMP + Tc, P0, RH) << endl;
    cout << "air pressure: " << xaGetAirPressureFromAltitude(h, -XA_ABSOLUTE_ZERO_TEMP + Tc, P0, RH, g) << endl;
    return 0;
}
 ```

+ Сопротивление среды (воздуха)

**xaGetAirResistanceForce** - Получить силу сопротивления воздуха при отсутствии подъемной силы (с учетом ветра)
 
+ Обработка данных

Примеры: *example\ordinary_least_squares_c*,*example\ordinary_least_squares_cpp*

**xaCalcOrdinaryLeastSquaresArray** - Определение коэффициентов линейной аппроксимации по МНК (1-го и 2-го порядка)

**xaGetLine** - Получить ```A1*X + A0```

**xaGetParabola** - Получить параболу ```A2*X^2 + A1*X + A0```

## Алгоритмы для C++
+ Обработка данных

**xaCalcOls** - Определение коэффициентов линейной аппроксимации по МНК (1-го и 2-го порядка)

**xaCalcLine** - Получить ```A1*X + A0``` или ```A2*X^2 + A1*X + A0``` в зависимости от набора коэффициентов

**xaKalmanFilterSimple1D** - Одномерный фильтр Калмана

## Полезные ссылки
http://www.graphicsgems.org/ - официальный онлайновый репозиторий для кода из серии книг « Графические драгоценные камни »

https://geographiclib.sourceforge.io/ - GeographicLib представляет собой небольшой набор классов C ++ для выполнения преобразований между географическими, UTM, ИБП, MGRS, геоцентрическими и локальными декартовыми координатами для расчетов гравитации (например, EGM2008), геоида и геомагнитного поля (например, WMM2010) и для решение геодезических задач.

https://habr.com/post/140274/ - одномерный фильтр Калмана

