#ifndef MADGWICK_FILTER_HPP_INCLUDED
#define MADGWICK_FILTER_HPP_INCLUDED

#include "math.h"

#define XA_BETA_DEF     0.515        /// 2 * пропорциональное усиление (стандартное для MPU60xx)

#ifdef XA_MADGWICK_FILTER_USE_DOUBLE
typedef double mFloat;
#else
typedef float mFloat;
#endif

class xaMadgwickFilter {
public:
    volatile mFloat beta;                            /**< 2 * пропорциональное усиление (Kp) */
    volatile mFloat q0, q1, q2, q3;                  /**< кватернион сенсорной рамки относительно вспомогательной рамки */
    mFloat sampleFreq;                               /**< частота дискретизации */
    mFloat period;                                   /**< период */

    xaMadgwickFilter() {};

    /** \brief Инициализировать фильтр Madgwick
     * \param _beta 2 * пропорциональное усиление (Kp)
     * \param _sampleFreq частота дискретизации
     */
    xaMadgwickFilter(mFloat _beta, mFloat _sampleFreq);

    /** \brief Обновление алгоритма AHRS с использованием магнитометра
     * \param obj класс фильтра Madgwick
     * \param gx данные от гироскопа по оси x
     * \param gy данные от гироскопа по оси y
     * \param gz данные от гироскопа по оси z
     * \param ax данные от акселерометра по оси x
     * \param ay данные от акселерометра по оси y
     * \param az данные от акселерометра по оси z
     * \param mx данные от магнитометра по оси x
     * \param my данные от магнитометра по оси y
     * \param mz данные от магнитометра по оси z
     */
    friend void MadgwickAHRSupdate(xaMadgwickFilter& obj, mFloat gx, mFloat gy, mFloat gz, mFloat ax, mFloat ay, mFloat az, mFloat mx, mFloat my, mFloat mz);

    /** \brief Обновление алгоритма AHRS без использованием магнитометра
     * \param obj класс фильтра Madgwick
     * \param gx данные от гироскопа по оси x
     * \param gy данные от гироскопа по оси y
     * \param gz данные от гироскопа по оси z
     * \param ax данные от акселерометра по оси x
     * \param ay данные от акселерометра по оси y
     * \param az данные от акселерометра по оси z
     */
    friend void MadgwickAHRSupdateIMU(xaMadgwickFilter& obj, mFloat gx, mFloat gy, mFloat gz, mFloat ax, mFloat ay, mFloat az);

    /** \brief Получить углы Тейт-Брайан
     * Определяем выходные переменные из обновленного кватерниона --- это углы Тейт-Брайан, обычно используемые в ориентации самолета.
     * В этой системе координат положительная ось z направлена вниз к Земле.
     * Углы Тейта-Брайана, а также углы Эйлера некоммутативны. То есть получить правильную ориентацию вращения должны быть применены в правильном порядке,
     * который для этой конфигурации рыскания, тангажа, а затем рулон.
     * For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
     * \param pitch это угол между осью X датчика и плоскостью заземления Земли, к Земле положителен, вверх к небу отрицателен.
     * \param yaw это угол между осью датчика X и магнитным севером Земли (или истинным севером, если скорректирован для локального склонения,
     * глядя вниз на положительный рыскак датчика против часовой стрелки.
     * \param roll это угол между осью Y датчика и землей, ось Y - положительный крен.
     */
    inline void getTaitBryanAngle(mFloat& pitch, mFloat& yaw, mFloat& roll) {
        static const mFloat PI = 3.14159265358979323846;
        static const mFloat PI_DIV180 = PI/180.0;
        yaw   = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        pitch = -asin(2.0 * (q1 * q3 - q0 * q2));
        roll  = atan2(2.0 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch *= 180.0 / PI;
        yaw   *= 180.0 / PI;
        roll  *= 180.0 / PI;
    }

    /** \brief Получить ускорения в глобальной системе координат
     * \param ax данные от акселерометра по оси x
     * \param ay данные от акселерометра по оси y
     * \param az данные от акселерометра по оси z
     * \param ax_global данные от акселерометра по оси x в глобальной системе координат
     * \param ay_global данные от акселерометра по оси y в глобальной системе координат
     * \param az_global данные от акселерометра по оси z в глобальной системе координат
     */
    inline void getAccGlobalCoorSystem(mFloat ax, mFloat ay, mFloat az, mFloat& ax_global, mFloat& ay_global, mFloat& az_global) {
        mFloat mtx[9];
        mtx[0] = 1.0f - 2.0f*q2*q2 - 2.0f*q3*q3;
        mtx[1] = 2.0f*q1*q2 - 2.0f*q3*q0;
        mtx[2] = 2.0f*q1*q3 + 2.0f*q2*q0;
        mtx[3] = 2.0f*q1*q2 + 2.0f*q3*q0;
        mtx[4] = 1.0f - 2.0f*q1*q1 - 2.0f*q3*q3;
        mtx[5] = 2.0f*q2*q3 - 2.0f*q1*q0;
        mtx[6] = 2.0f*q1*q3 - 2.0f*q2*q0;
        mtx[7] = 2.0f*q2*q3 + 2.0f*q1*q0;
        mtx[8] = 1.0f - 2.0f*q1*q1 - 2.0f*q2*q2;

        ax_global = mtx[0] * ax + mtx[1] * ay + mtx[2] * az;
        ay_global = mtx[3] * ax + mtx[4] * ay + mtx[5] * az;
        az_global = mtx[6] * ax + mtx[7] * ay + mtx[8] * az;
    }
};

#endif // MADGWICK_FILTER_HPP_INCLUDED
