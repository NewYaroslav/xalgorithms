#ifndef ORDINARY_LEAST_SQUARES_H_INCLUDED
#define ORDINARY_LEAST_SQUARES_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

enum xaOlsFunctionType {
    XA_EQUATION_LINE = 0,           /**< Y = A1*X + A0 */
    XA_EQUATION_PARABOLA = 1,       /**< Y = A2*X^2 + A1*X + A0 */
};

/** \brief Метод наименьших квадратов
 * Определение коэффициентов линейной аппроксимации по МНК
 * С помощью данной функции можно найти коэффициенты для функций
 * Y = A1*X + A0 или Y = A2*X^2 + A1*X + A0
 * \param point двумерный массив точек [x, y][n]
 * \param n количество точек
 * \param coeff массив коэффициентов (2 либо 3 коэффициента)
 * \param line_type тип линии
 */
void xaCalcOrdinaryLeastSquaresArray(double (*point)[2], int n, double* coeff, int line_type);

/** \brief Получить A1*X + A0
 * \param coeff коэффициенты A0 и A1
 * \param x переменная X
 * \return значение Y
 */
inline double xaGetLine(double* coeff, double x) {
    return coeff[1] * x + coeff[0];
}

/** \brief Получить параболу A2*X^2 + A1*X + A0
 * \param coeff коэффициенты A0, A1, A2
 * \param x переменная X
 * \return значение Y
 */
inline double xaGetParabola(double* coeff, double x) {
    return coeff[2] * x * x + coeff[1] * x + coeff[0];
}

#ifdef __cplusplus
}
#endif

#endif // ORDINARY_LEAST_SQUARES_H_INCLUDED
