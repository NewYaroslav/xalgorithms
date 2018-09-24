#include <stdio.h>
#include <stdlib.h>
#include "ordinary_least_squares.h"

int main()
{
    printf("Hello world!\n");
    double pointP[10][2];
    double pointL[10][2];
    double coeffP[3];
    double coeffL[3];
    const NUM_POINT = 10;

    int i;
    for(i = 0; i < NUM_POINT; ++i) {
        pointL[i][0] = (double)i;
        pointL[i][1] = 8*(double)i - 3 + ((rand()%100)-50)*0.05;
        //printf("x = %f y = %f\n",pointL[i][0],pointL[i][1]);
    }
    pointP[0][0] = -3.2;
    pointP[0][1] = -6.7;
    pointP[1][0] = -2.2;
    pointP[1][1] = -2.9;
    pointP[2][0] = -1.51;
    pointP[2][1] = 1.1;
    pointP[3][0] = 0.4;
    pointP[3][1] = 4.3;
    pointP[4][0] = 1.2;
    pointP[4][1] = 4.2;
    pointP[5][0] = 2.1;
    pointP[5][1] = 4.5;
    pointP[6][0] = 3.4;
    pointP[6][1] = 5.2;
    pointP[7][0] = 4.2;
    pointP[7][1] = 8.4;
    pointP[8][0] = 5.5;
    pointP[8][1] = 9.6;
    pointP[9][0] = 6.6;
    pointP[9][1] = 20.0;

    xaCalcOrdinaryLeastSquaresArray(pointP, NUM_POINT, coeffP, XA_EQUATION_PARABOLA);
    printf("coeff parabola a0 = %f a1 = %f a2 = %f\n",coeffP[0],coeffP[1],coeffP[2]);

    xaCalcOrdinaryLeastSquaresArray(pointP, NUM_POINT, coeffL, XA_EQUATION_LINE);
    printf("coeff line a0 = %f a1 = %f\n",coeffL[0],coeffL[1]);

    for(i = 0; i < 10; ++i) {
        double yp = xaGetParabola(coeffP, pointP[i][0]);
        double yl = xaGetLine(coeffL, pointP[i][0]);
        printf("x = %f parabola y = %f line y = %f\n",pointP[i][0],yp, yl);
    }
    return 0;
}
