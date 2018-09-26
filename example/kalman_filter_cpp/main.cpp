#include <iostream>
#include "kalman_filter.hpp"
#include <vector>

using namespace std;

int main()
{
    cout << "Hello world!" << endl;
    xaKalmanFilterSimple1D<double> KalmanFilter(15.0d, 1.0d);

    std::vector<double> vData;
    for(int i = 0; i < 100; ++i) {
        double data = 8*(double)i - 3 + ((rand()%100)-50)*0.05;
        vData.push_back(data);
    }

    KalmanFilter.set_state(vData[0], 0.1);
    for(int i = 1; i < 100; ++i) {
        cout << 8*(double)i - 3 << " -> filter -> " << KalmanFilter.updata(vData[i]) << endl;
    }

    return 0;
}
