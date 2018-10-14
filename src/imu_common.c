#include "imu_common.h"

void xaGetRotationMatrixF32(float q0, float q1, float q2, float q3, float* mtx) {
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

void xaTransformRelativeToAbsoluteF32(float* lx, float* mtx, float* gx) {
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

void xaGetTaitBryanAngleF32(float q[4], float* pitch, float* yaw, float* roll) {
    static const float PI = 3.14159265358979323846f;
    static const float COEFF = 180.0f / PI;
    float q0q0 = q[0] * q[0];
    float q1q1 = q[1] * q[1];
    float q2q2 = q[2] * q[2];
    float q3q3 = q[3] * q[3];
    *yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q0q0 + q1q1 - q2q2 - q3q3);
    *pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q0q0 - q1q1 - q2q2 + q3q3);
    *pitch *= COEFF;
    *yaw   *= COEFF;
    *roll  *= COEFF;
}

void xaGetTaitBryanAngleF64(double q[4], double* pitch, double* yaw, double* roll) {
    static const double PI = 3.14159265358979323846f;
    static const double COEFF = 180.0f / PI;
    double q0q0 = q[0] * q[0];
    double q1q1 = q[1] * q[1];
    double q2q2 = q[2] * q[2];
    double q3q3 = q[3] * q[3];
    *yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q0q0 + q1q1 - q2q2 - q3q3);
    *pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q0q0 - q1q1 - q2q2 + q3q3);
    *pitch *= COEFF;
    *yaw   *= COEFF;
    *roll  *= COEFF;
}

void xaToGlobalCoordinatesF32(float q[4], float local[3], float global[3]) {
    mFloat mtx[9];
    float x2q1q0 = 2.0f * q[1] * q[0];
    float x2q1q1 = 2.0f * q[1] * q[1];
    float x2q1q2 = 2.0f * q[1] * q[2];
    float x2q1q3 = 2.0f * q[1] * q[3];
    float x2q2q0 = 2.0f * q[2] * q[0];
    float x2q2q2 = 2.0f * q[2] * q[2];
    float x2q2q3 = 2.0f * q[2] * q[3];
    float x2q3q0 = 2.0f * q[3] * q[0];
    float x2q3q3 = 2.0f * q[3] * q[3];
    mtx[0] = 1.0f - x2q2q2 - x2q3q3;
    mtx[1] = x2q1q2 - x2q3q0;
    mtx[2] = x2q1q3 + x2q2q0;
    mtx[3] = x2q1q2 + x2q3q0;
    mtx[4] = 1.0f - x2q1q1 - x2q3q3;
    mtx[5] = x2q2q3 - x2q1q0;
    mtx[6] = x2q1q3 - x2q2q0;
    mtx[7] = x2q2q3 + x2q1q0;
    mtx[8] = 1.0f - x2q1q1 - x2q2q2;

    global[0] = mtx[0] * local[0] + mtx[1] * local[1] + mtx[2] * local[2];
    global[1] = mtx[3] * local[0] + mtx[4] * local[1] + mtx[5] * local[2];
    global[2] = mtx[6] * local[0] + mtx[7] * local[1] + mtx[8] * local[2];
}

void xaToGlobalCoordinatesF64(double q[4], double local[3], double global[3]) {
    mFloat mtx[9];
    double x2q1q0 = 2.0f * q[1] * q[0];
    double x2q1q1 = 2.0f * q[1] * q[1];
    double x2q1q2 = 2.0f * q[1] * q[2];
    double x2q1q3 = 2.0f * q[1] * q[3];
    double x2q2q0 = 2.0f * q[2] * q[0];
    double x2q2q2 = 2.0f * q[2] * q[2];
    double x2q2q3 = 2.0f * q[2] * q[3];
    double x2q3q0 = 2.0f * q[3] * q[0];
    double x2q3q3 = 2.0f * q[3] * q[3];
    mtx[0] = 1.0f - x2q2q2 - x2q3q3;
    mtx[1] = x2q1q2 - x2q3q0;
    mtx[2] = x2q1q3 + x2q2q0;
    mtx[3] = x2q1q2 + x2q3q0;
    mtx[4] = 1.0f - x2q1q1 - x2q3q3;
    mtx[5] = x2q2q3 - x2q1q0;
    mtx[6] = x2q1q3 - x2q2q0;
    mtx[7] = x2q2q3 + x2q1q0;
    mtx[8] = 1.0f - x2q1q1 - x2q2q2;

    global[0] = mtx[0] * local[0] + mtx[1] * local[1] + mtx[2] * local[2];
    global[1] = mtx[3] * local[0] + mtx[4] * local[1] + mtx[5] * local[2];
    global[2] = mtx[6] * local[0] + mtx[7] * local[1] + mtx[8] * local[2];
}

