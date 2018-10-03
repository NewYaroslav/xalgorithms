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
inline void xaGetRotationMatrix(float q0, float q1, float q2, float q3, float* mtx) {
    mtx[0] = 1.0f - 2.0f*q2*q2 - 2.0f*q3*q3;
    mtx[1] = 2.0f*q1*q2 - 2.0f*q3*q0;
    mtx[2] = 2.0f*q1*q3 + 2.0f*q2*q0;
    mtx[3] = 2.0f*q1*q2 + 2.0f*q3*q0;
    mtx[4] = 1.0f - 2.0f*q1*q1 - 2.0f*q3*q3;
    mtx[5] = 2.0f*q2*q3 - 2.0f*q1*q0;
    mtx[6] = 2.0f*q1*q3 - 2.0f*q2*q0;
    mtx[7] = 2.0f*q2*q3 + 2.0f*q1*q0;
    mtx[8] = 1.0f - 2.0f*q1*q1 - 2.0f*q2*q2;
}

/** \brief Получить абсолютные значения вектора из относительных
 * Длина всех векторов равна 3
 * \param lx вектор в локальными координатах
 * \param mtx матрица поворота
 * \param gx вектор в глобальных координатах
 */
inline void xaTransformRelativeToAbsolute(float* lx, float* mtx, float* gx) {
    gx[0] = mtx[0] * lx[0] + mtx[1] * lx[1] + mtx[2] * lx[2];
    gx[1] = mtx[3] * lx[0] + mtx[4] * lx[1] + mtx[5] * lx[2];
    gx[2] = mtx[6] * lx[0] + mtx[7] * lx[1] + mtx[8] * lx[2];
}

#ifdef __cplusplus
}
#endif

#endif // IMU_COMMON_H_INCLUDED
