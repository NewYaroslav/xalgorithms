#include <stdio.h>
#include <stdlib.h>
#include "fast_matrix.h"

double _xaFastDeterminantMatrix4x4(double a[16]) {
    double a5a10 = a[5]*a[10];
    double a13a6 = a[13]*a[6];
    double a9a7 = a[9]*a[7];
    double a13a10 = a[13]*a[10];
    double a9a6 = a[9]*a[6];
    double a5a11 = a[5]*a[11];
    double a8a5 = a[8]*a[5];
    double a4a9 = a[4]*a[9];
    double a4a11 = a[4]*a[11];
    double a8a7 = a[8]*a[7];
    double detA =
        a[0] * (a5a10*a[15] + a13a6*a[11] + a9a7*a[14]
        - a13a10*a[7] - a9a6*a[15] - a5a11*a[14]) -
        a[1] * (a[4]*a[10]*a[15] + a[12]*a[6]*a[11] + a8a7*a[14]
        - a[12]*a[10]*a[7] - a4a11*a[13] - a[8]*a[6]*a[15]) +
        a[2] * (a4a9*a[15] + a[12]*a5a11 + a8a7*a[13]
        - a[12]*a9a7 - a4a11*a[13] - a8a5*a[15]) -
        a[3] * (a4a9*a[14] + a[8]*a13a6 + a[12]*a5a10
        - a[12]*a9a6 - a[4]*a13a10 - a8a5*a[14]);

    return detA;
}

int main()
{
    double mat[16] = {-1.0,4.0,5.0,1.0,
                5.0,3.0,2.0,1.0,
                3.0,-3.0,3.0,3.0,
                5.0,5.0,5.0,5.0};
    double detA = xaFastDeterminantMatrix4x4F64(mat);
    printf("detA %f\n", detA);
    xaFastInverseMatrix4x4F64(mat, mat);

    for(int y = 0; y < 4; ++y) {
        for(int x = 0; x < 4; ++x) {
            printf("%f ", mat[x + y*4]);
        }
        printf("\n");
    }

    double a[16] = {
        3,3,3,3,
        1,2,2,2,
        6,6,6,6,
        7,7,7,7
    };
    double b[16] = {
        5,5,5,5,
        2,2,3,4,
        1,9,9,9,
        8,8,8,8,
    };
    double c[16];
    xaFastMultiplicationMatrix4x4F64(a, b, c);

    for(int y = 0; y < 4; ++y) {
        for(int x = 0; x < 4; ++x) {
            printf("%f ", c[x + y*4]);
        }
        printf("\n");
    }
    return 0;
}
