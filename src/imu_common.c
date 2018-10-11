#include "imu_common.h"

void xaGetRotationMatrix(float q0, float q1, float q2, float q3, float* mtx) {
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

void xaTransformRelativeToAbsolute(float* lx, float* mtx, float* gx) {
    gx[0] = mtx[0] * lx[0] + mtx[1] * lx[1] + mtx[2] * lx[2];
    gx[1] = mtx[3] * lx[0] + mtx[4] * lx[1] + mtx[5] * lx[2];
    gx[2] = mtx[6] * lx[0] + mtx[7] * lx[1] + mtx[8] * lx[2];
}

void xaGetRotationMatrixF64(double q0, double q1, double q2, double q3, double* mtx) {
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

void xaTransformRelativeToAbsoluteF64(double* lx, double* mtx, double* gx) {
    gx[0] = mtx[0] * lx[0] + mtx[1] * lx[1] + mtx[2] * lx[2];
    gx[1] = mtx[3] * lx[0] + mtx[4] * lx[1] + mtx[5] * lx[2];
    gx[2] = mtx[6] * lx[0] + mtx[7] * lx[1] + mtx[8] * lx[2];
}

void xaMultiplyQuaternionF64(double qa[4], double qb[4], double qc[4]) {
    /* q1 * q2 = s1s2 + s2q1 + s1q2 - v1 * v2 + v1 x v2
        v1 * v2 = a1a2 + b1b2 + c1c2
        v1 x v2 = (b1c2 - c1b2)i + (c1a2-a1c2)j + (a1b2-b1a2)k
    */
    qc[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
    qc[1] = qb[0]*qa[1] + qa[0]*qb[1] + (qa[2]*qb[3] - qa[3]*qb[2]);
    qc[2] = qb[0]*qa[2] + qa[0]*qb[2] + (qa[3]*qb[1] - qa[1]*qb[3]);
    qc[3] = qb[0]*qa[3] + qa[0]*qb[3] + (qa[1]*qb[2] - qa[2]*qb[1]);
}

void xaMultiplyQuaternionF32(float qa[4], float qb[4], float qc[4]) {
    /* q1 * q2 = s1s2 + s2q1 + s1q2 - v1 * v2 + v1 x v2
        v1 * v2 = a1a2 + b1b2 + c1c2
        v1 x v2 = (b1c2 - c1b2)i + (c1a2-a1c2)j + (a1b2-b1a2)k
    */
    qc[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
    qc[1] = qb[0]*qa[1] + qa[0]*qb[1] + (qa[2]*qb[3] - qa[3]*qb[2]);
    qc[2] = qb[0]*qa[2] + qa[0]*qb[2] + (qa[3]*qb[1] - qa[1]*qb[3]);
    qc[3] = qb[0]*qa[3] + qa[0]*qb[3] + (qa[1]*qb[2] - qa[2]*qb[1]);
}
