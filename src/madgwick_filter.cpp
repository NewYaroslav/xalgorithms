#include "madgwick_filter.hpp"

//=====================================================================================================
// madgwick_filter.cpp
//=====================================================================================================
//
// Реализация алгоритмов Madgwick's IMU и AHRS.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Дата         Автор               Заметки
// 29/09/2011   SOH Madgwick        Начальная версия
// 02/10/2011   SOH Madgwick        Оптимизирован для снижения загрузки процессора
// 19/02/2012   SOH Madgwick        Измерение магнитометра нормируется
// 21/04/2017   Yaroslav Barabanov  исправлен баг с магнитометром
// 27/09/2017   Yaroslav Barabanov  доработан класс xaMadgwickFilter
// 11/10/2018   Yaroslav Barabanov  добавлена поддержка double
//
//=====================================================================================================

#ifdef XA_MADGWICK_FILTER_USE_DOUBLE
static const mFloat PI = 3.1415926535897932384626433832795d;
static const mFloat PI_DIV180 = PI/180.0d;
#else
static const mFloat PI = 3.14159265358979323846f;
static const mFloat PI_DIV180 = PI/180.0f;
#endif

static mFloat invSqrt(mFloat x);

xaMadgwickFilter::xaMadgwickFilter(mFloat _beta, mFloat _sampleFreq) {
    beta = _beta;
    q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
    sampleFreq = _sampleFreq;
    period = 1.0 / sampleFreq;
}

//---------------------------------------------------------------------------------------------------
// Обновление алгоритма AHRS
void MadgwickAHRSupdate(
    xaMadgwickFilter& obj,
    mFloat gx, mFloat gy, mFloat gz,
    mFloat ax, mFloat ay, mFloat az,
    mFloat mx, mFloat my, mFloat mz) {
    //
    volatile mFloat& q0 = obj.q0; volatile mFloat& q1 = obj.q1;
    volatile mFloat& q2 = obj.q2; volatile mFloat& q3 = obj.q3;
    volatile mFloat& beta = obj.beta;
    mFloat& period = obj.period;

    mFloat recipNorm;
    mFloat s0, s1, s2, s3;
    mFloat qDot1, qDot2, qDot3, qDot4;
    mFloat hx, hy;
    mFloat _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    mFloat _4bx, _4bz;
    mFloat _8bx, _8bz;
    mFloat _2q0, _2q1, _2q2, _2q3;//, _2q0q2, _2q2q3;
    mFloat q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Использовать алгоритм IMU, если измерение магнитометра недействительно
    // (избегает NaN при нормализации магнитометра)
    if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
        MadgwickAHRSupdateIMU(obj, gx, gy, gz, ax, ay, az);
        return;
    }

    // Скорость изменения кватерниона с гироскопа
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    // Вычислить обратную связь, только если выполняется измерение акселерометра
    // (избегает NaN в нормализации акселерометра)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        // Нормализовать измерение акселерометра
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Нормализовать измерение магнитометра
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Вспомогательные переменные, чтобы избежать повторной арифметики
        _2q0mx = 2.0 * q0 * mx;
        _2q0my = 2.0 * q0 * my;
        _2q0mz = 2.0 * q0 * mz;
        _2q1mx = 2.0 * q1 * mx;
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        //_2q0q2 = 2.0f * q0 * q2;
        //_2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2
        + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;

        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1
        + my * q2q2 + _2q2 * mz * q3 - my * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);

        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1
        + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;

        _4bx = 2.0 * _2bx;
        _4bz = 2.0 * _2bz;
        _8bx = 2.0 * _4bx;
        _8bz = 2.0 * _4bz;

        // Корректирующий шаг алгоритма спуска градиента
        s0 = -_2q2 * (2.0 * (q1q3 - q0q2) - ax) + _2q1 * (2.0 * (q0q1 + q2q3) - ay) + -_4bz * q2 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx)
            + (-_4bx * q3 + _4bz * q1) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q2 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0 * (q1q3 - q0q2) - ax) + _2q0 * (2.0 * (q0q1 + q2q3) - ay) + -4.0 * q1 * (2.0 * (0.5 - q1q1 - q2q2) - az) + _4bz * q3 * (_4bx * (0.5 - q2q2 - q3q3)
            + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q2 + _4bz * q0) * (_4bx * (q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my) + (_4bx * q3 - _8bz * q1)*(_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0 * (q1q3 - q0q2) - ax) + _2q3 * (2.0 * (q0q1 + q2q3) - ay) + (-4.0 * q2) * (2.0 * (0.5 - q1q1 - q2q2) - az) + (-_8bx * q2 - _4bz * q0) * (_4bx*(0.5 - q2q2 - q3q3)
            + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q1 + _4bz * q3) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q0-_8bz * q2) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0 * (q1q3 - q0q2) - ax) + _2q2 * (2.0 * (q0q1 + q2q3) - ay) + (-_8bx * q3 + _4bz * q1) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx)
            + (-_4bx * q0 + _4bz * q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my)+(_4bx * q1)*(_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // нормализовать величину шага
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Применить шаг обратной связи
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Интегрируйте скорость изменения кватерниона, чтобы получить кватернион
    q0 += qDot1 * period;
    q1 += qDot2 * period;
    q2 += qDot3 * period;
    q3 += qDot4 * period;

    // Нормализовать кватернион
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU обновление алгоритма
void MadgwickAHRSupdateIMU(
    xaMadgwickFilter& obj,
    mFloat gx, mFloat gy, mFloat gz,
    mFloat ax, mFloat ay, mFloat az) {
    //
    volatile mFloat& q0 = obj.q0; volatile mFloat& q1 = obj.q1;
    volatile mFloat& q2 = obj.q2; volatile mFloat& q3 = obj.q3;
    volatile mFloat& beta = obj.beta;
    mFloat& period = obj.period;

    mFloat recipNorm;
    mFloat s0, s1, s2, s3;
    mFloat qDot1, qDot2, qDot3, qDot4;
    mFloat _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Скорость изменения кватерниона с гироскопа
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    // Вычислить обратную связь только в том случае, если выполняется измерение акселерометра (избегает NaN в нормализации акселерометра)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        // Нормализовать измерение акселерометра
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Вспомогательные переменные, чтобы избежать повторной арифметики
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        _4q0 = 4.0 * q0;
        _4q1 = 4.0 * q1;
        _4q2 = 4.0 * q2;
        _8q1 = 8.0 * q1;
        _8q2 = 8.0 * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Корректирующий шаг алгоритма спуска градиента
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // нормализовать величину шага
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Применить шаг обратной связи
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Интегрируйте скорость изменения кватерниона, чтобы получить кватернион
    q0 += qDot1 * period;
    q1 += qDot2 * period;
    q2 += qDot3 * period;
    q3 += qDot4 * period;

    // Нормализовать кватернион
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Быстрый обратный квадратный корень
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static mFloat invSqrt(mFloat x) {
    mFloat halfx = 0.5 * x;
    mFloat y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(mFloat*)&i;
    y = y * (1.5 - (halfx * y * y));
    return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
