#ifndef IMU_COMMON_H_INCLUDED
#define IMU_COMMON_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Получить матриу поворота из квартениона
 * \param q0 переменная квартениона
 * \param q1 переменная квартениона
 * \param q2 переменная квартениона
 * \param q3 переменная квартениона
 * \param mtx матрица поворота
 */
extern inline void xaGetRotationMatrixF32(float q0, float q1, float q2, float q3, float* mtx);

/** \brief Получить матриу поворота из квартениона
 * \param q0 переменная квартениона
 * \param q1 переменная квартениона
 * \param q2 переменная квартениона
 * \param q3 переменная квартениона
 * \param mtx матрица поворота
 */
extern inline void xaGetRotationMatrixF64(double q0, double q1, double q2, double q3, double* mtx);

/** \brief Получить абсолютные значения вектора из относительных
 * Длина всех векторов равна 3
 * \param lx вектор в локальными координатах
 * \param mtx матрица поворота
 * \param gx вектор в глобальных координатах
 */
extern inline void xaTransformRelativeToAbsoluteF32(float* lx, float* mtx, float* gx);

/** \brief Получить абсолютные значения вектора из относительных
 * Длина всех векторов равна 3
 * \param lx вектор в локальными координатах
 * \param mtx матрица поворота
 * \param gx вектор в глобальных координатах
 */
extern inline void xaTransformRelativeToAbsoluteF64(double* lx, double* mtx, double* gx);

/** \brief Умножение квартенионов
 * Умножение квартенионов позволяет "повернуть" первый квартенион на второй
 * \param qa первый квартенион
 * \param qb второй квартенион
 * \param qc итоговый квартенион
 */
extern inline void xaMultiplyQuaternionF64(double qa[4], double qb[4], double qc[4]);

/** \brief Умножение квартенионов
 * Умножение квартенионов позволяет "повернуть" первый квартенион на второй
 * \param qa первый квартенион
 * \param qb второй квартенион
 * \param qc итоговый квартенион
 */
extern inline void xaMultiplyQuaternionF32(float qa[4], float qb[4], float qc[4]);

/** \brief Получить углы Тейт-Брайан
 * \param q квартенион
 * Определяем выходные переменные из обновленного кватерниона --- это углы Тейт-Брайан, обычно используемые в ориентации самолета.
 * В этой системе координат положительная ось z направлена вниз к Земле.
 * Углы Тейта-Брайана, а также углы Эйлера некоммутативны. То есть получить правильную ориентацию вращения должны быть применены в правильном порядке,
 * который для этой конфигурации рыскания, тангажа, а затем рулон.
 * For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
 * \param pitch это угол между осью X датчика и плоскостью заземления Земли, к Земле положителен, вверх к небу отрицателен.
 * \param yaw это угол между осью датчика X и магнитным севером Земли (или истинным севером, если скорректирован для локального склонения,
 * глядя вниз на положительный рыскак датчика против часовой стрелки.
 * \param roll это угол между осью Y датчика и землей, ось Y - положительный крен.
 */
extern inline void xaGetTaitBryanAngleF32(float q[4], float* pitch, float* yaw, float* roll);

/** \brief Получить углы Тейт-Брайан
 * \param q квартенион
 * Определяем выходные переменные из обновленного кватерниона --- это углы Тейт-Брайан, обычно используемые в ориентации самолета.
 * В этой системе координат положительная ось z направлена вниз к Земле.
 * Углы Тейта-Брайана, а также углы Эйлера некоммутативны. То есть получить правильную ориентацию вращения должны быть применены в правильном порядке,
 * который для этой конфигурации рыскания, тангажа, а затем рулон.
 * For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
 * \param pitch это угол между осью X датчика и плоскостью заземления Земли, к Земле положителен, вверх к небу отрицателен.
 * \param yaw это угол между осью датчика X и магнитным севером Земли (или истинным севером, если скорректирован для локального склонения,
 * глядя вниз на положительный рыскак датчика против часовой стрелки.
 * \param roll это угол между осью Y датчика и землей, ось Y - положительный крен.
 */
extern inline void xaGetTaitBryanAngleF64(double q[4], double* pitch, double* yaw, double* roll);

/** \brief Получить переменные в глобальной системе координат
 * \param q квартенион
 * \param local данные в локальной системе координат (x,y,z)
 * \param global данные в глобальной системе координат (x,y,z)
 */
extern inline void xaToGlobalCoordinatesF32(float q[4], float local[3], float global[3]);

/** \brief Получить переменные в глобальной системе координат
 * \param q квартенион
 * \param local данные в локальной системе координат (x,y,z)
 * \param global данные в глобальной системе координат (x,y,z)
 */
extern inline void xaToGlobalCoordinatesF64(double q[4], double local[3], double global[3]);

#ifdef __cplusplus
}
#endif

#endif // IMU_COMMON_H_INCLUDED
