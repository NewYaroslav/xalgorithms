#include <iostream>
#include "ordinary_least_squares.hpp"

using namespace std;

struct point2D {
    double x;
    double y;
    point2D(double xIn, double yIn) {
        x = xIn;
        y = yIn;
    }
};

int main()
{
    cout << "Hello world!" << endl;
    std::vector<point2D> vPoint;
    const int NUM_POINT = 10;
    for(int i = 0; i < NUM_POINT; ++i) {
        vPoint.push_back(point2D((double)i, 8*(double)i - 3 + ((rand()%100)-50)*0.05));
    }

    std::vector<double> vCoeff;
    xaCalcOls(vPoint, vCoeff, XA_PARABOLA);
    for(size_t i = 0; i < vCoeff.size(); ++i) {
        cout << "a[" << i << "] = " << vCoeff[i] << endl;
    }
    return 0;
}
