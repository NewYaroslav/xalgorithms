#include "ordinary_least_squares.h"
#include <stdio.h>

void xaCalcOrdinaryLeastSquaresArray(double (*point)[2], int n, double* coeff, int line_type) {
    if(line_type == XA_EQUATION_LINE) {
        double sumx = 0;
        double sumy = 0;
        double sumx2 = 0;
        double sumxy = 0;
        int i;
        for (i = 0; i < n; i++) {
            sumx += point[i][0];
            sumy += point[i][1];
            sumx2 += point[i][0] * point[i][0];
            sumxy += point[i][0] * point[i][1];
        }
        coeff[1] = ((double)n * sumxy - (sumx * sumy)) / ((double)n * sumx2 - sumx * sumx);
        coeff[0] = (sumy - coeff[0] * sumx) / (double)n;
    } else
    if(line_type == XA_EQUATION_PARABOLA) {
        double sx = 0;
        double sy = 0;
        double sx2 = 0;
        double sx3 = 0;
        double sx4 = 0;
        double sxy = 0;
        double sx2y = 0;
        int i;
        for (i = 0; i < n; ++i) {
            sx += point[i][0];
            sy += point[i][1];
            double m2 = point[i][0] * point[i][0];
            double m3 = m2 * point[i][0];
            sx2 += m2;
            sx3 += m3;
            sx4 += m3 * point[i][0];
            double mxy = point[i][0] * point[i][1];
            sxy += mxy;
            sx2y += point[i][0] * mxy;
        }
        double sxsx2 = sx*sx2;
        double sxsx4 = sx*sx4;
        double sx2sx2 = sx2*sx2;
        double sx2sx3 = sx2*sx3;
        double sxsx3 = sx*sx3;
        double nsx3 = (double)n*sx3;
        /* найдем определитель матрицы
         * n   sx  sx2
         * sx  sx2 sx3
         * sx2 sx3 sx4
         */
        double A = (double)n * (sx2 * sx4 - sx3 * sx3) - sx* (sx * sx4 - sx2 * sx3) + sx2 * (sx * sx3 - sx2 * sx2);
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
        coeff[0] = A * ((sx2*sx4 - sx3*sx3) * sy + (sx2sx3 - sxsx4) * sxy + (sxsx3 - sx2sx2) * sx2y);
        coeff[1] = A * ((sx2sx3 - sxsx4) * sy + ((double)n*sx4 - sx2sx2) * sxy + (sxsx2 - nsx3) * sx2y);
        coeff[2] = A * ((sxsx3 - sx2sx2) * sy + (sxsx2 - nsx3) * sxy + ((double)n*sx2 - sx*sx) * sx2y);
    }
}
