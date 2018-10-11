#ifndef IMU_COMMON_H_INCLUDED
#define IMU_COMMON_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Получить матриу поворота из квартениона
 * \param q0
 * \param q1
 * \param q2
 * \param q3
 * \param mtx матрица поворота
 */
extern inline void xaGetRotationMatrix(float q0, float q1, float q2, float q3, float* mtx);

/** \brief Получить матриу поворота из квартениона
 * \param q0
 * \param q1
 * \param q2
 * \param q3
 * \param mtx матрица поворота
 */
extern inline void xaGetRotationMatrixF64(double q0, double q1, double q2, double q3, double* mtx);

/** \brief Получить абсолютные значения вектора из относительных
 * Длина всех векторов равна 3
 * \param lx вектор в локальными координатах
 * \param mtx матрица поворота
 * \param gx вектор в глобальных координатах
 */
extern inline void xaTransformRelativeToAbsolute(float* lx, float* mtx, float* gx);

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

#ifdef __cplusplus
}
#endif

#endif // IMU_COMMON_H_INCLUDED
