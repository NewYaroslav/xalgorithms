#ifndef ORDINARY_LEAST_SQUARES_HPP_INCLUDED
#define ORDINARY_LEAST_SQUARES_HPP_INCLUDED

#include <vector>

enum xaOlsFunctionType {
    XA_LINE = 0,           /**< Y = A1*X + A0 */
    XA_PARABOLA = 1,       /**< Y = A2*X^2 + A1*X + A0 */
};

template <typename T1, typename T2>
void xaCalcOls(std::vector<T1>& point, std::vector<T2>& coeff, int typeLine) {
        if(typeLine == XA_LINE) {
        double sx = 0, sy = 0, sx2 = 0, sxy = 0;
        for (size_t i = 0; i < point.size(); ++i) {
            sx += point[i].x;
            sy += point[i].y;
            sx2 += point[i].x * point[i].x;
            sxy += point[i].x * point[i].y;
        }
        coeff.resize(2);
        coeff[1] = ((double)point.size() * sxy - (sx * sy)) / ((double)point.size() * sx2 - sx * sx);
        coeff[0] = (sy - coeff[0] * sx) / (double)point.size();
    } else
    if(typeLine == XA_PARABOLA) {
        double sx = 0, sy = 0, sx2 = 0, sx3 = 0, sx4 = 0, sxy = 0, sx2y = 0;
        for (size_t i = 0; i < point.size(); ++i) {
            sx += point[i].x;
            sy += point[i].y;
            double m2 = point[i].x * point[i].x;
            double m3 = m2 * point[i].x;
            sx2 += m2;
            sx3 += m3;
            sx4 += m3 * point[i].x;
            double mxy = point[i].x * point[i].y;
            sxy += mxy;
            sx2y += point[i].x * mxy;
        }
        double sxsx2 = sx*sx2;
        double sxsx4 = sx*sx4;
        double sx2sx2 = sx2*sx2;
        double sx2sx3 = sx2*sx3;
        double sxsx3 = sx*sx3;
        double nsx3 = (double)point.size()*sx3;
        /* найдем определитель матрицы
         * n   sx  sx2
         * sx  sx2 sx3
         * sx2 sx3 sx4
         */
        double A = (double)point.size() * (sx2 * sx4 - sx3 * sx3) - sx* (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx3 - sx2 * sx2);
        A = 1.0/A;
        /* найдем транспонированную матрицу, она будет такой же
         * n   sx  sx2
         * sx  sx2 sx3
         * sx2 sx3 sx4
         * далее найдем определитель для матриц 2 x 2 и применим матрицу кофакторов
         * sx2*sx4-sx3*sx3  sx2*sx3-sx*sx4  sx*sx3-sx2*sx2
         * sx3*sx2-sx*sx4   n*sx4-sx2*sx2   sx*sx2-n*sx3
         * sx*sx3-sx2*sx2   sx*sx2-n*sx3    n*sx2-sx*sx
         * далее каждый элемент надо делить на определитель
         */
        coeff.resize(3);
        coeff[0] = A * ((sx2*sx4 - sx3*sx3) * sy + (sx2sx3 - sxsx4) * sxy + (sxsx3 - sx2sx2) * sx2y);
        coeff[1] = A * ((sx2sx3 - sxsx4) * sy + ((double)point.size()*sx4 - sx2sx2) * sxy + (sxsx2 - nsx3) * sx2y);
        coeff[2] = A * ((sxsx3 - sx2sx2) * sy + (sxsx2 - nsx3) * sxy + ((double)point.size()*sx2 - sx*sx) * sx2y);
    }
}

template <typename T1, typename T2, typename T3>
T1 xaCalcLine(std::vector<T2>& coeff, T3 x) {
    if(coeff.size() == 2) {
        return coeff[1] * x + coeff[0];
    } else
    if(coeff.size() == 3) {
        return coeff[2] * x * x + coeff[1] * x + coeff[0];
    }
    return 0;
}

#endif // ORDINARY_LEAST_SQUARES_HPP_INCLUDED
