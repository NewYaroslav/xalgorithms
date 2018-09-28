#include "fast_matrix.h"

double xaFastDeterminantMatrix4x4F64(double* a) {
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

float xaFastDeterminantMatrix4x4F32(float* a) {
    float a5a10 = a[5]*a[10];
    float a13a6 = a[13]*a[6];
    float a9a7 = a[9]*a[7];
    float a13a10 = a[13]*a[10];
    float a9a6 = a[9]*a[6];
    float a5a11 = a[5]*a[11];
    float a8a5 = a[8]*a[5];
    float a4a9 = a[4]*a[9];
    float a4a11 = a[4]*a[11];
    float a8a7 = a[8]*a[7];
    float detA =
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

bool xaFastInverseMatrix4x4F64(double* a, double* b) {
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
    double a10a15 = a[10]*a[15];
    double a12a10 = a[12]*a[10];
    double a6a15 = a[6]*a[15];
    double a12a6 = a[12]*a[6];

    double A11 = a5a10*a[15] + a13a6*a[11] + a9a7*a[14]
        - a13a10*a[7] - a9a6*a[15] - a5a11*a[14];
    double A12 = a[4]*a10a15 + a12a6*a[11] + a8a7*a[14]
        - a12a10*a[7] - a4a11*a[13] - a[8]*a6a15;
    double A13 = a4a9*a[15] + a[12]*a5a11 + a8a7*a[13]
        - a[12]*a9a7 - a4a11*a[13] - a8a5*a[15];
    double A14 = a4a9*a[14] + a[8]*a13a6 + a[12]*a5a10
        - a[12]*a9a6 - a[4]*a13a10 - a8a5*a[14];
    double detA = a[0] * A11 - a[1] * A12 + a[2] * A13 - a[3] * A14;
    if(detA == 0.0) return false;

    detA = 1.0/detA;

    double a2a11 = a[2]*a[11];
    double a14a3 = a[14]*a[3];
    double a2a15 = a[2]*a[15];
    double a13a2 = a[13]*a[2];
    double a13a3 = a[13]*a[3];
    double a0a13 = a[0]*a[13];
    double a12a9 = a[12]*a[9];
    double a12a1 = a[12]*a[1];
    double a12a5 = a[12]*a[5];
    double a4a1 = a[4]*a[1];
    double a4a13 = a[4]*a[13];
    double a6a3 = a[6]*a[3];
    double a6a11 = a[6]*a[11];
    double a10a3 = a[10]*a[3];
    double a1a10 = a[1]*a[10];
    double a2a7 = a[2]*a[7];
    double a0a9 = a[0]*a[9];
    double a8a1 = a[8]*a[1];
    double a0a5 = a[0]*a[5];
    double a14a11 = a[14]*a[11];

    double A21 = a1a10*a[15] + a[13]*a2a11 + a[9]*a14a3
        - a13a10*a[3] - a[1]*a14a11 - a[9]*a2a15;
    double A22 = a[0]*a10a15 + a[12]*a2a11 + a[8]*a14a3
        - a12a10*a[3] - a[8]*a2a15 - a[0]*a14a11;
    double A23 = a0a9*a[14] + a[8]*a13a3 + a12a1*a[11]
        - a12a9*a[3] -  a0a13*a[11] - a[8]*a[1]*a[15];
    double A24 = a0a9*a[14] + a[8]*a13a2 + a12a1*a[10]
        - a12a9*a[2] - a0a13*a[10] - a[8]*a[1]*a[14];

    double A31 = a[1]*a6a15 + a[5]*a14a3 + a13a2*a[7]
        - a[13]*a6a3 - a[1]*a[14]*a[7] - a[5]*a2a15;
    double A32 = a[0]*a6a15 + a[4]*a13a3 + a[12]*a2a7
        - a12a6*a[3] - a0a13*a[7] - a[4]*a2a15;
    double A33 = a0a5*a[15] + a12a1*a[7] + a4a13*a[3]
        - a12a5*a[3] - a0a13*a[7] - a4a1*a[15];
    double A34 = a0a5*a[14] + a4a13*a[2] + a12a1*a[6]
        - a12a5*a[2] - a0a13*a[6] - a4a1*a[13];

    double A41 = a[1]*a6a11 + a[5]*a10a3 + a[9]*a2a7
        - a[9]*a6a3 - a1a10*a[7] - a[5]*a2a11;
    double A42 = a[0]*a6a11 + a[4]*a10a3 + a[8]*a2a7
        - a[8]*a6a3 - a[0]*a[10]*a[7] - a[4]*a2a11;
    double A43 = a0a5*a[11] + a4a9*a[3] + a8a1*a[7]
        - a8a5*a[3] - a0a9*a[7] - a4a1*a[11];
    double A44 = a[0]*a5a10 + a8a1*a[6] + a4a9*a[2]
        - a8a5*a[2] - a0a9*a[6] - a4a1*a[10];

    b[0] = A11*detA; a[1] = -A21*detA; b[2] = A31*detA; b[3] = -A41*detA;
    b[4] = -A12*detA; a[5] = A22*detA; b[6] = -A32*detA; b[7] = A42*detA;
    b[8] = A13*detA; a[9] = -A23*detA; b[10] = A33*detA; b[11] = -A43*detA;
    b[12] = -A14*detA; a[13] = A24*detA; b[14] = -A34*detA; b[15] = A44*detA;
    return true;
}

bool xaFastInverseMatrix4x4F32(float* a, float* b) {
    float a5a10 = a[5]*a[10];
    float a13a6 = a[13]*a[6];
    float a9a7 = a[9]*a[7];
    float a13a10 = a[13]*a[10];
    float a9a6 = a[9]*a[6];
    float a5a11 = a[5]*a[11];
    float a8a5 = a[8]*a[5];
    float a4a9 = a[4]*a[9];
    float a4a11 = a[4]*a[11];
    float a8a7 = a[8]*a[7];
    float a10a15 = a[10]*a[15];
    float a12a10 = a[12]*a[10];
    float a6a15 = a[6]*a[15];
    float a12a6 = a[12]*a[6];

    float A11 = a5a10*a[15] + a13a6*a[11] + a9a7*a[14]
        - a13a10*a[7] - a9a6*a[15] - a5a11*a[14];
    float A12 = a[4]*a10a15 + a12a6*a[11] + a8a7*a[14]
        - a12a10*a[7] - a4a11*a[13] - a[8]*a6a15;
    float A13 = a4a9*a[15] + a[12]*a5a11 + a8a7*a[13]
        - a[12]*a9a7 - a4a11*a[13] - a8a5*a[15];
    float A14 = a4a9*a[14] + a[8]*a13a6 + a[12]*a5a10
        - a[12]*a9a6 - a[4]*a13a10 - a8a5*a[14];
    float detA = a[0] * A11 - a[1] * A12 + a[2] * A13 - a[3] * A14;
    if(detA == 0.0) return false;

    detA = 1.0/detA;

    float a2a11 = a[2]*a[11];
    float a14a3 = a[14]*a[3];
    float a2a15 = a[2]*a[15];
    float a13a2 = a[13]*a[2];
    float a13a3 = a[13]*a[3];
    float a0a13 = a[0]*a[13];
    float a12a9 = a[12]*a[9];
    float a12a1 = a[12]*a[1];
    float a12a5 = a[12]*a[5];
    float a4a1 = a[4]*a[1];
    float a4a13 = a[4]*a[13];
    float a6a3 = a[6]*a[3];
    float a6a11 = a[6]*a[11];
    float a10a3 = a[10]*a[3];
    float a1a10 = a[1]*a[10];
    float a2a7 = a[2]*a[7];
    float a0a9 = a[0]*a[9];
    float a8a1 = a[8]*a[1];
    float a0a5 = a[0]*a[5];
    float a14a11 = a[14]*a[11];

    float A21 = a1a10*a[15] + a[13]*a2a11 + a[9]*a14a3
        - a13a10*a[3] - a[1]*a14a11 - a[9]*a2a15;
    float A22 = a[0]*a10a15 + a[12]*a2a11 + a[8]*a14a3
        - a12a10*a[3] - a[8]*a2a15 - a[0]*a14a11;
    float A23 = a0a9*a[14] + a[8]*a13a3 + a12a1*a[11]
        - a12a9*a[3] -  a0a13*a[11] - a[8]*a[1]*a[15];
    float A24 = a0a9*a[14] + a[8]*a13a2 + a12a1*a[10]
        - a12a9*a[2] - a0a13*a[10] - a[8]*a[1]*a[14];

    float A31 = a[1]*a6a15 + a[5]*a14a3 + a13a2*a[7]
        - a[13]*a6a3 - a[1]*a[14]*a[7] - a[5]*a2a15;
    float A32 = a[0]*a6a15 + a[4]*a13a3 + a[12]*a2a7
        - a12a6*a[3] - a0a13*a[7] - a[4]*a2a15;
    float A33 = a0a5*a[15] + a12a1*a[7] + a4a13*a[3]
        - a12a5*a[3] - a0a13*a[7] - a4a1*a[15];
    float A34 = a0a5*a[14] + a4a13*a[2] + a12a1*a[6]
        - a12a5*a[2] - a0a13*a[6] - a4a1*a[13];

    float A41 = a[1]*a6a11 + a[5]*a10a3 + a[9]*a2a7
        - a[9]*a6a3 - a1a10*a[7] - a[5]*a2a11;
    float A42 = a[0]*a6a11 + a[4]*a10a3 + a[8]*a2a7
        - a[8]*a6a3 - a[0]*a[10]*a[7] - a[4]*a2a11;
    float A43 = a0a5*a[11] + a4a9*a[3] + a8a1*a[7]
        - a8a5*a[3] - a0a9*a[7] - a4a1*a[11];
    float A44 = a[0]*a5a10 + a8a1*a[6] + a4a9*a[2]
        - a8a5*a[2] - a0a9*a[6] - a4a1*a[10];

    b[0] = A11*detA; a[1] = -A21*detA; b[2] = A31*detA; b[3] = -A41*detA;
    b[4] = -A12*detA; a[5] = A22*detA; b[6] = -A32*detA; b[7] = A42*detA;
    b[8] = A13*detA; a[9] = -A23*detA; b[10] = A33*detA; b[11] = -A43*detA;
    b[12] = -A14*detA; a[13] = A24*detA; b[14] = -A34*detA; b[15] = A44*detA;
    return true;
}

void xaFastMultiplicationMatrix4x4F64(double* a, double* b, double* c) {
    c[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    c[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    c[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
    c[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];
    //
    c[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
    c[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
    c[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
    c[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];
    //
    c[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
    c[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
    c[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
    c[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];
    //
    c[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
    c[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
    c[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    c[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];
}

void xaFastMultiplicationMatrix4x4F32(float* a, float* b, float* c) {
    c[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    c[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    c[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
    c[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];
    //
    c[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
    c[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
    c[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
    c[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];
    //
    c[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
    c[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
    c[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
    c[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];
    //
    c[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
    c[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
    c[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    c[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];
}
