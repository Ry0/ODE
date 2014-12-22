#include <iostream>
#include <cmath>
#include <fstream>
#include <string>

using namespace std;

int main(void){
  double P[3];
  std::ofstream ofs("./hert.dat");

  for (int i = 0; i < 10000; ++i){
    // P[0] = 0.4;
    // P[1] = 0.1+0.16*pow(sin(0.01*i),3);
    // P[2] = 0.3 + 0.13*cos(0.01*i) - 0.05*cos(2*0.01*i) - 0.02*cos(3*0.01*i) - 0.01*cos(4*0.01*i);
    P[0] = 0.08 * sin(0.01 * i) + 0.1;
    P[1] = 0.1 * sin(0.01 * i);
    P[2] = 0.1 * sin(0.03 * i + 3.14/4.0) + 0.2;
    ofs << P[0] << "\t" << P[1] << "\t" << P[2] << endl;
  }

}