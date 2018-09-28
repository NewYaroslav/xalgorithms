#ifndef FAST_MATRIX_H_INCLUDED
#define FAST_MATRIX_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#else
#include <stdbool.h>
#endif

/** \brief Получить определитель матрицы
 * \param а матрица
 */
extern inline double xaFastDeterminantMatrix4x4F64(double* a);

/** \brief Получить определитель матрицы
 * \param а матрица
 */
extern inline float xaFastDeterminantMatrix4x4F32(float* a);

/** \brief Получить обратную матрицу
 * \param а матрица для обработки
 * \param b обратная матрица (A^-1)
 */
extern inline bool xaFastInverseMatrix4x4F64(double* a, double* b);

/** \brief Получить обратную матрицу
 * \param а матрица для обработки
 * \param b обратная матрица (A^-1)
 */
extern inline bool xaFastInverseMatrix4x4F32(float* a, float* b);

/** \brief Быстрое умножение матриц
 * Функция находит A*B, обе матрицы должны быть 4x4
 * \param a матрица
 * \param b матрица
 * \param c матрица полученная в результате умножения
 * \return
 */
extern inline void xaFastMultiplicationMatrix4x4F64(double* a, double* b, double* c);

/** \brief Быстрое умножение матриц
 * Функция находит A*B, обе матрицы должны быть 4x4
 * \param a матрица
 * \param b матрица
 * \param c матрица полученная в результате умножения
 * \return
 */
extern inline void xaFastMultiplicationMatrix4x4F32(float* a, float* b, float* c);

#ifdef __cplusplus
}
#endif

#endif // FAST_MATRIX_H_INCLUDED
