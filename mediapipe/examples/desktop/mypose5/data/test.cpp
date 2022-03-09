#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>
using namespace std;

void mkdir_output(const string &output_path){
    if (access(output_path.c_str(), 0) == -1) {
        // mkdir(output_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        system(("mkdir "+output_path).c_str());
        std::cout<<output_path<<" create! "<<std::endl;
    }
    else {
        system(("rm -rf "+output_path).c_str());
        system(("mkdir "+output_path).c_str());
        std::cout<<output_path<<" recover create! "<<std::endl;
    }
}


int main(){
    mkdir_output("fold1/fold1_2");
}