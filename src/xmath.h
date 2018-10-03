#ifndef XMATH_H_INCLUDED
#define XMATH_H_INCLUDED

#ifdef	__cplusplus
extern "C" {
#endif

/** \brief Решить квадратичное уравнение
 * Решает квадратичное уравнение ax ^ 2 + bx + c = 0. Решения помещаются в 'x'.
 * \param a коэффициент
 * \param b коэффициент
 * \param c коэффициент
 * \param x массив всех решений
 * \return количество решений
 */
extern inline unsigned int xa_quadratic_solve(double a, double b, double c, double x[2]);

/** \brief Интерполирует линейную функцию, определяемую двумя точками.
 * \param x Укажите, кто имеет значение «y», которое мы ищем для функции.
 * \param x1 Координата X первой точки
 * \param y1 Координата Y первой точки
 * \param x2 Координата X второй точки
 * \param y2 Координата Y второй точки
 * \return Искомое значение Y
 */
extern inline double xa_linear_interpolation(double x, double x1, double y1, double x2, double y2);

#ifdef	__cplusplus
}
#endif

#endif // XMATH_H_INCLUDED
