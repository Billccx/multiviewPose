#include <iostream>
#include <cmath>
#include <vector>
using namespace std;
int main(){
    cout<<sin(3.1415926/2.0)<<endl;
    const std::vector<double> coeffs0={0.171335,-0.528704,-0.00208366,2.20354e-05,0.495262,600.764,600.986,331.948,248.697};
    for(auto item: coeffs0){
        cout<<item<<endl;
    }
    cout<<coeffs0[3]*10000<<endl;
}