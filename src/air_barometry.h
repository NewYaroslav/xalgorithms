#ifndef AIR_BAROMETRY_H_INCLUDED
#define AIR_BAROMETRY_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

#define XA_ABSOLUTE_ZERO_TEMP -273.15   /// температура абсолютного нуля
#define XA_TECHNICAL_ATMOSPHERE 101325  /// давление в 1 атмосферу

/** \brief Получить молярную массу воздуха
 * \param P давление
 * \param Pv давление водяного пара
 * \return молярная масса воздуха
 */
double xaGetAirMolarMass(double P, double Pv);

/** \brief Получить плотность воздуха
 * \param T абсолютная температура (в Кельвинах)
 * \param P абсолютное давление (в Паскалях)
 * \param RH относительная влажность (0 - 100)
 * \return плотность воздуха (кг/м3)
 */
double xaGetAirDensity(double T, double P, double RH);

/** \brief Получить давление на высоте над уровнем моря
 * Зависимость давления газа от высоты (барометрическая формула)
 * \param h высота
 * \param T абсолютная температура (в Кельвинах)
 * \param P0 давление на высоте уровня моря (в Паскалях)
 * \param g ускорение свободного падения (м/сс)
 * \return давление
 */
double xaGetAirPressureFromAltitude(double h, double T, double P0, double RH, double g);

#ifdef __cplusplus
}
#endif

#endif // AIR_BAROMETRY_H_INCLUDED
