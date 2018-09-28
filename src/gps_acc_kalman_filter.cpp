#include "gps_acc_kalman_filter.hpp"
#include "fast_matrix.h"

void xaGpsAccKalmanFilter::predict(double ax, double ay) {
/*  Xk  -   вектор состояния системы
    x   -   координата x
    y   -   координата y
    vx  -   скорость по оси x
    vy  -   скорость по оси y

    Fk - матрица перехода между состояниями
    1   0   dt  0
    0   1   0   dt
    0   0   1   0
    0   0   0   1

    Xk-1    - состояние системы в прошлый момент времени
    x0   -   координата x
    y0   -   координата y
    vx0  -   скорость по оси x
    vy0  -   скорость по оси y

    B - матрица применения управдяющего воздействия
    dt^2/2  0
    0       dt2^2/2
    dt      0
    0       dt

    uk-1 - управляющее воздействие в прошлый момент времени
    Это проекции ускорений в абсолютной системе координат
    ax0
    ay0
*/
    // Предсказание состояния системы
    x = x0 + vx0 * dt + ax * dt2div2;
    y = y0 + vy0 * dt + ay * dt2div2;
    vx = vx0 + ax * dt;
    vy = vy0 + ay * dt;
    // Предсказание ошибки коварации

    // для ковариационной матрицы шума процесса Q надем переменные
    predict_count++;
    vd = accDev * predict_count;
    vd2 = vd * vd;
    pd = vd * predict_count / 2.0;
    cd = vd * pd;
    pd2 = pd * pd;
    //
    double temp1 = Pk0[2] + dt * Pk0[10];
    double temp2 = Pk0[3] + dt * Pk0[11];
    //
    Pk[0] = Pk0[0] + dt * Pk0[8] + dt * temp1 + pd2;
    Pk[1] = Pk0[1] + dt * Pk0[9] + dt * temp2;
    Pk[2] = temp1 + cd;
    Pk[3] = temp2;
    //
    temp1 = Pk0[6] + dt * Pk0[14];
    temp2 = Pk0[7] + dt * Pk0[15];
    Pk[4] = Pk0[4] + dt * Pk0[12] + dt * temp1;
    Pk[5] = Pk0[5] + dt * Pk0[13] + dt * temp2 + pd2;
    Pk[6] = temp1;
    Pk[7] = temp2 + cd;
    //
    Pk[8] = Pk0[1] + dt * Pk0[10] + cd;
    Pk[9] = Pk0[9] + dt * Pk0[11];
    Pk[10] = Pk0[10] + vd2;
    Pk[11] = Pk0[11];
    //
    Pk[12] = Pk0[12] + dt * Pk0[14];
    Pk[13] = Pk0[13] + dt * Pk0[15] + cd;
    Pk[14] = Pk0[14];
    Pk[15] = Pk0[15] + vd2;
    // запомним
    x0 = x;
    y0 = y;
    vx0 = vx;
    vy0 = vy;
}

void xaGpsAccKalmanFilter::updata(double _x, double _y, double _vx, double _vy, double posSigma2, double velSigma2) {
    predict_count = 0;
    /*  выражение Pk * H равно Pk, так как матрица H единичная
        Kk = Pk * (Pk + R) ^-1;
        матрица R
        Sigma^2pos  0   0   0
        0   Sigma^2pos  0   0
        0   0   Sigma^2vel  0
        0   0   0   Sigma^2vel
    */
    double Kk[16]; // усиление Калмана
    double mt[16]; // матрица для промежуточных вычислений
    //
    mt[0] = Pk[0] + posSigma2;
    mt[1] = Pk[1];
    mt[2] = Pk[2];
    mt[3] = Pk[3];
    //
    mt[4] = Pk[4];
    mt[5] = Pk[5] + posSigma2;
    mt[6] = Pk[6];
    mt[7] = Pk[7];
    //
    mt[8] = Pk[8];
    mt[9] = Pk[9];
    mt[10] = Pk[10] + velSigma2;
    mt[11] = Pk[11];
    //
    mt[12] = Pk[12];
    mt[13] = Pk[13];
    mt[14] = Pk[14];
    mt[15] = Pk[15] + velSigma2;
    //

    xaFastInverseMatrix4x4F64(mt, mt); // найдем обратную матрицу
    xaFastMultiplicationMatrix4x4F64(Pk, mt, Kk);
    /*  матрица Zk
        _x
        _y
        _vx
        _vy
    */
    // найдем Xk
    x0 = x + Kk[0] * (_x - x) + Kk[1] * (_y - y) + Kk[2] * (_vx - vx) + Kk[3] * (_vy - vy);
    y0 = y + Kk[4] * (_x - x) + Kk[5] * (_y - y) + Kk[6] * (_vx - vx) + Kk[7] * (_vy - vy);
    vx0 = vx + Kk[8] * (_x - x) + Kk[9] * (_y - y) + Kk[10] * (_vx - vx) + Kk[11] * (_vy - vy);
    vy0 = vy + Kk[12] * (_x - x) + Kk[13] * (_y - y) + Kk[14] * (_vx - vx) + Kk[15] * (_vy - vy);
    // найдем Pk
        //
    mt[0] = 1.0 - Kk[0];
    mt[1] = -Kk[1];
    mt[2] = -Kk[2];
    mt[3] = -Kk[3];
    //
    mt[4] = -Kk[4];
    mt[5] = 1.0 - Kk[5];
    mt[6] = -Kk[6];
    mt[7] = -Kk[7];
    //
    mt[8] = -Kk[8];
    mt[9] = -Kk[9];
    mt[10] = 1.0 - Kk[10];
    mt[11] = -Kk[11];
    //
    mt[12] = -Kk[12];
    mt[13] = -Kk[13];
    mt[14] = -Kk[14];
    mt[15] = 1.0 - Kk[15];
    //
    xaFastMultiplicationMatrix4x4F64(mt, Pk, Pk0);
}
