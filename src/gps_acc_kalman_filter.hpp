#ifndef GPS_ACC_KALMAN_FILTER_HPP_INCLUDED
#define GPS_ACC_KALMAN_FILTER_HPP_INCLUDED

class xaGpsAccKalmanFilter {
private:
    double x, y;                    /**< координаты */
    double vx, vy;                  /**< скорости */
    double x0, y0;                  /**< координаты в прошлый момент времени */
    double vx0, vy0;                /**< скорости в прошлый момент времени */
    //double ax0, ay0;                /**< ускорения в абсолютной системе координат в прошлый момент времени */
    double dt;                      /**< время */
    double dt2;                     /**< время в квадрате */
    double dt2div2;                 /**< время в квадрате деленое на 2 */

    double accDev;                  /**< ошибка акселерометра */
    unsigned long predict_count;    /**<  */
    /* Ковариационная матрица шума процесса
        vd = accDev * predict_count
        pd = vd * predict_count / 2
        cd = vd * pd
        pd2 = pd * pd
    */
    double vd, pd, cd, vd2, pd2;              /**< Ковариационная матрица шума процесса */
    double Pk[16];                  /**< Матрица предсказание ошибки */
    double Pk0[16];                 /**< Матрица предсказание ошибки (прошлые значения) */
public:

    xaGpsAccKalmanFilter() {};

    /** \brief
     * \param _x начальные координаты, ось x
     * \param _y начальные координаты, ось y
     * \param _vx начальная скорость, ось x
     * \param _vy начальная скорость, ось y
     * \param _accDev ошибка акселерометра
     * \param _posDev ошибка GPS
     * \param период обновления данных
     */
    xaGpsAccKalmanFilter(double _x, double _y,
                         double _vx, double _vy,
                         double _accDev, double _posDev,
                         double _dt) {
        x0 = _x;
        y0 = _y;
        vx0 = _vx;
        vy0 = _vy;
        accDev = _accDev;
        dt = _dt;
        dt2 = dt*dt;
        dt2div2 = dt2/2.0;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                Pk0[i] = i == j ? _posDev : 0.0;
            }
        }
    }

    void predict(double ax, double ay);

    /** \brief
     * \param x координата по оси x
     * \param y координата по оси y
     * \param vx скорость по оси x
     * \param vy скорость по оси y
     */
    void updata(double x, double y, double vx, double vy, double posSigma2, double velSigma2);
};

#endif // GPS_ACC_KALMAN_FILTER_HPP_INCLUDED
