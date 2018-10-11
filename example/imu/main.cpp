#include <iostream>
#include "imu_common.h"

using namespace std;

int main() {
    float mtx[9]; // матрица вращения
    xaGetRotationMatrix(0,1,0,0,mtx); // повернем на 180 градусов вокруг оси X
    float lacc[3] = {0,0,1}; // вектор, где z = 1 в локальной системе координат
    float racc[3]; // вектор в глобальной системе координат
    // получить значение вектора в глобальной системе координат из значений в локальной системе координат
    xaTransformRelativeToAbsolute(lacc, mtx, racc);
    cout << racc[0] << " " << racc[1] << " " << racc[2] << endl;

    double R = 1.0;
    std::cout << 3.1415 / R << std::endl;

    return 0;
}
