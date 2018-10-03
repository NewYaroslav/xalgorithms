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
