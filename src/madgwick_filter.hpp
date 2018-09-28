#ifndef MADGWICK_FILTER_HPP_INCLUDED
#define MADGWICK_FILTER_HPP_INCLUDED

#include "math.h"

#define XA_BETA_DEF     0.515f        /// 2 * пропорциональное усиление (стандартное для MPU60xx)

class xaMadgwickFilter {
public:
    volatile float beta;                            /**< 2 * пропорциональное усиление (Kp) */
    volatile float q0, q1, q2, q3;                  /**< кватернион сенсорной рамки относительно вспомогательной рамки */
    float sampleFreq;                               /**< частота дискретизации */
    float period;                                   /**< период */

    xaMadgwickFilter() {};

    /** \brief Инициализировать фильтр Madgwick
     * \param _beta 2 * пропорциональное усиление (Kp)
     * \param _sampleFreq частота дискретизации
     */
    xaMadgwickFilter(float _beta, float _sampleFreq);

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
    friend void MadgwickAHRSupdate(xaMadgwickFilter& obj, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    /** \brief Обновление алгоритма AHRS без использованием магнитометра
     * \param obj класс фильтра Madgwick
     * \param gx данные от гироскопа по оси x
     * \param gy данные от гироскопа по оси y
     * \param gz данные от гироскопа по оси z
     * \param ax данные от акселерометра по оси x
     * \param ay данные от акселерометра по оси y
     * \param az данные от акселерометра по оси z
     */
    friend void MadgwickAHRSupdateIMU(xaMadgwickFilter& obj, float gx, float gy, float gz, float ax, float ay, float az);

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
    inline void getTaitBryanAngle(float& pitch, float& yaw, float& roll) {
        static const float PI = 3.14159265358979323846f;
        static const float PI_DIV180 = PI/180.0f;
        yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
        roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;
    }

    /** \brief Получить ускорения в глобальной системе координат
     * \param ax данные от акселерометра по оси x
     * \param ay данные от акселерометра по оси y
     * \param az данные от акселерометра по оси z
     * \param ax_global данные от акселерометра по оси x в глобальной системе координат
     * \param ay_global данные от акселерометра по оси y в глобальной системе координат
     * \param az_global данные от акселерометра по оси z в глобальной системе координат
     */
    inline void getAccGlobalCoorSystem(float ax, float ay, float az, float& ax_global, float& ay_global, float& az_global) {
        float mtx[9];
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
