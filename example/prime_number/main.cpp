#include <iostream>
#include "bpsw.hpp"

using namespace std;

int main()
{
    const int a = 19, b = 20;
    if(xaBPSW::isprime(a)) {
        cout << a <<  " is prime" << endl;
    }
    if(xaBPSW::isprime(b)) {
        cout << b << " is prime" << endl;
    }
    return 0;
}
