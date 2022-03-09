#include <iostream>
#include <chrono>
#include <string>
#include <algorithm>
using namespace std;
int main(){
    auto tp1 = std::chrono::steady_clock::now();
    int i=0;
    while(i<10000){
        i++;
    }
    auto tp2 = std::chrono::steady_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1).count() << "microseconds" << std::endl;
}