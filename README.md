## xalgorithms
Сборник различных алгоритмов на C и C++

## Как установить\использовать
Код представлен в виде *.h .hpp* и *.c .cpp* файлов. Добавьте файлы в свой проект.

Все функции сборника алгоритмов начинаются с *xa*, чтобы избежать конфликтов со сторонними библиотеками.

## Алгоритмы для C и C++
+ Работа с геоданными

Примеры: *example\geo_distance*,*example\geo_acceleration_gravity_latitude*
файлы: *src\geo_common.h*,*src\geo_geohash.h*,*src\geo_great_circle_distance.h*,*src\geo_great_circle_distance.c*,*src\geo_vincentys_formulae.h*,*src\geo_vincentys_formulae.c*,*src\geo_acceleration_gravity.h*

**xaGetGeoDistanceUsingVincentysFormulae** - расстояние между двумя точками географического местоположения используя формулу Винченти

**xaGetGeoDistanceUsingGreatCircleDistance** - расстояние между двумя точками географического местоположения используя расстояние по большому кругу

**xaGeohashEncode** - Кодирует географическое местоположение в Geohash

**xaGeohashComparePoints** - Сравнить две точки географического местоположения используя Geohash

**xaDMStoDD** - Преобразовать градусы, минуты и секунды в десятичные градусы

**xaGetAccelerationGravityLatitude** - Получить ускорение свободного падения в зависимости от широты

+ Барометрия

Примеры: *example\air*
файлы: *src\air_barometry.h*,*src\air_barometry.c*

**xaGetAirDensity** - Получить плотность воздуха

**xaGetAirPressureFromAltitude** - Получить давление на высоте над уровнем моря

+ Сопротивление среды (воздуха)
Примеры: *example\air*
файлы: *src\air_damping.h*,*src\air_damping.c*

**xaGetAirResistanceForce** - Получить силу сопротивления воздуха при отсутствии подъемной силы (с учетом ветра)
 
+ Обработка данных

Примеры: *example\ordinary_least_squares_c*

Файлы: *src\ordinary_least_squares.h*,*src\ordinary_least_squares.c*

**xaCalcOrdinaryLeastSquaresArray** - Определение коэффициентов линейной аппроксимации по МНК (1-го и 2-го порядка)

**xaGetLine** - Получить ```A1*X + A0```

**xaGetParabola** - Получить параболу ```A2*X^2 + A1*X + A0```

## Алгоритмы для C++
+ Обработка данных

Примеры: *example\ordinary_least_squares_cpp*

Файлы: *src\ordinary_least_squares.hpp*

**xaCalcOls** - Определение коэффициентов линейной аппроксимации по МНК (1-го и 2-го порядка)

**xaCalcLine** - Получить ```A1*X + A0``` или ```A2*X^2 + A1*X + A0``` в зависимости от набора коэффициентов

**xaKalmanFilterSimple1D** - Одномерный фильтр Калмана

+ Фильтры для IMU

Файлы: *src\madgwick_filter.hpp*, *src\madgwick_filter.cpp*

**xaMadgwickFilter** - Класс Madgwick фильтра.

+ Математика

**xaBPSW::isprime** - Алгоритм Бэйли-Померанс-Селфридж-Вагстафф (BPSW) проверки n на простоту

**xaFastInverseMatrix4x4** - Быстрое нахождение обратной матрицы 4x4

## Полезные ссылки
http://www.graphicsgems.org/ - официальный онлайновый репозиторий для кода из серии книг « Графические драгоценные камни »

https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml - Всемирная магнитная модель и связанное с ней программное обеспечение

https://geographiclib.sourceforge.io/ - GeographicLib представляет собой небольшой набор классов C ++ для выполнения преобразований между географическими, UTM, ИБП, MGRS, геоцентрическими и локальными декартовыми координатами для расчетов гравитации (например, EGM2008), геоида и геомагнитного поля (например, WMM2010) и для решение геодезических задач.

https://habr.com/post/140274/ - одномерный фильтр Калмана

http://e-maxx.ru/algo/bpsw - Тест BPSW на простоту чисел
