#include <iostream>
#include <math.h>
#include <fstream>
#include <ostream>
using namespace std;

double P[3] = {1.34, 0, 0.905};             // 先端の位置
double a[3];
double b[3] = {0.0, 0.0, 1.0};
double T[2] = {M_PI, 0.0};
double THETA[10] = {0.0};  // 関節の目標角度[rad]
double CalTheta[7] = {0.0};       // 目標角度計算用
double tmpTHETA3, tmpTHETA5;
double l[10] = {0.10, 0.10, 0.32, 0.435, 0.435, 0.235, 0.51, 0.51, 0.1, 0.1};   // リンクの長さ[m]
double min_theta[7] = {-180.0*M_PI/180.0,
                      -135.0*M_PI/180.0,
                       -45.0*M_PI/180.0,
                       -90.0*M_PI/180.0,
                      -360.0*M_PI/180.0,
                      -125.0*M_PI/180.0,
                      -360.0*M_PI/180.0}; // 各関節の最小角度[rad]
double max_theta[7] = { 180.0*M_PI/180.0,
                        65.0*M_PI/180.0,
                       120.0*M_PI/180.0,
                        70.0*M_PI/180.0,
                       360.0*M_PI/180.0,
                       125.0*M_PI/180.0,
                       360.0*M_PI/180.0};     // 各関節の最大角度[rad]
double max_thetaE = 0.0, min_thetaE = 0.0;

void CheckThetaE(){
  int CheckAnswer = 0;

    for (int i = 0; i < 7; ++i){
      if(min_theta[i] <= CalTheta[i] && CalTheta[i] <= max_theta[i]){
        CheckAnswer += 1;
      }else{
        cout << i <<"番目範囲外" << endl;
      }
    }

    if(CheckAnswer == 7){
      THETA[1] = CalTheta[0];
      THETA[3] = CalTheta[1];
      THETA[4] = CalTheta[2];
      THETA[5] = CalTheta[3];
      THETA[7] = CalTheta[4];
      THETA[8] = CalTheta[5];
      THETA[9] = CalTheta[6];
    }
  cout << "THETA_S = " << THETA[1]*180/M_PI << endl;
  cout << "THETA_L = " << THETA[3]*180/(M_PI) << endl;
  cout << "THETA_E = " << THETA[4]*180/(M_PI) << endl;
  cout << "THETA_U = " << THETA[5]*180/(M_PI) << endl;
  cout << "THETA_R = " << THETA[7]*180/(M_PI) << endl;
  cout << "THETA_B = " << THETA[8]*180/(M_PI) << endl;
  cout << "THETA_T = " << THETA[9]*180/(M_PI) << endl;
  cout << endl;
  cout << "THETA_S = " << CalTheta[0]*180/M_PI << endl;
  cout << "THETA_L = " << CalTheta[1]*180/M_PI << endl;
  cout << "THETA_E = " << CalTheta[2]*180/M_PI << endl;
  cout << "THETA_U = " << CalTheta[3]*180/M_PI << endl;
  cout << "THETA_R = " << CalTheta[4]*180/M_PI << endl;
  cout << "THETA_B = " << CalTheta[5]*180/M_PI << endl;
  cout << "THETA_T = " << CalTheta[6]*180/M_PI << endl;
  cout << endl;
}


// 逆運動学
void  inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];

  double l2;
  double VirtualTHETA;

  l2 = 2 * l[3] * sin((M_PI - CalTheta[2])/2);
  // cout << "l2 =" << l2 << endl;
  VirtualTHETA = CalTheta[2]/2;
  // std::cout << l2 << std::endl;
  double P5x = Px - (l[8] + l[9])*a[0];
  double P5y = Py - (l[8] + l[9])*a[1];
  double P5z = Pz - (l[8] + l[9])*a[2];


  // printf("Target  Position: x=%6.3f y=%6.3f z=%6.3f \n", Px, Py, Pz);

  double tmpL  = sqrt(P5x * P5x + P5y * P5y);
  double P1P   = sqrt((P5z - l[0] - l[1])*(P5z - l[0] - l[1]) + pow((tmpL - l[2]),2));
  // cout << "P1P = " << P1P << endl;
  double Ca    = (l2 * l2 + P1P * P1P - l[5] * l[5] - (l[6]+l[7]) * (l[6]+l[7]) )/(2 * l2 * P1P);  // cosα
  // cout << "Ca = " << Ca << endl;
  double phi   = atan2(P5z - (l[0]+l[1]), tmpL - l[2]);                      //φ
  // cout << "Phi = " << phi << endl;
  double alpha = atan2(sqrt(1 - Ca * Ca), Ca);                         //α
  // cout << "alpha = " << alpha << endl;

  double Cb    = (l2 * l2 + l[5] * l[5] + (l[6]+l[7])*(l[6]+l[7]) - P1P * P1P)/(2 * l2 * sqrt(l[5]*l[5] + (l[6]+l[7])*(l[6]+l[7])));  //cosβ
  // cout << "Cb = " << Cb << endl;
  double beta  = atan2(sqrt(1- Cb * Cb), Cb);                          //β
  // cout << "beta =" << beta << endl;
  double gamma = atan2(l[6]+l[7], l[5]);                                    //γ


  CalTheta[0] = atan2(P5y, P5x);
  tmpTHETA3 = - phi - alpha;
  CalTheta[1] = tmpTHETA3 - VirtualTHETA;
  tmpTHETA5 = M_PI - beta - gamma;
  CalTheta[3] = tmpTHETA5 - VirtualTHETA;

  a2[0] = -cos(tmpTHETA3+tmpTHETA5)*(a[0]*cos(CalTheta[0])+a[1]*sin(CalTheta[0])) + a[2]*sin(tmpTHETA3+tmpTHETA5);
  a2[1] = -a[0]*sin(CalTheta[0]) + a[1]*cos(CalTheta[0]);
  a2[2] = -sin(tmpTHETA3+tmpTHETA5)*(a[0]*cos(CalTheta[0])+a[1]*sin(CalTheta[0])) - a[2]*cos(tmpTHETA3+tmpTHETA5);
  b2[0] = -cos(tmpTHETA3+tmpTHETA5)*(b[0]*cos(CalTheta[0])+b[1]*sin(CalTheta[0])) + b[2]*sin(tmpTHETA3+tmpTHETA5);
  b2[1] = -b[0]*sin(CalTheta[0]) + b[1]*cos(CalTheta[0]);
  b2[2] = -sin(tmpTHETA3+tmpTHETA5)*(b[0]*cos(CalTheta[0])+b[1]*sin(CalTheta[0])) - b[2]*cos(tmpTHETA3+tmpTHETA5);

  CalTheta[4] = atan2(a2[1], a2[0]);

  CalTheta[5] = atan2(cos(CalTheta[4]) * a2[0] + sin(CalTheta[4]) * a2[1], a2[2]);
  CalTheta[6] = atan2(sin(CalTheta[4]) * sin(CalTheta[4])*b2[0] - cos(CalTheta[4])*sin(CalTheta[4])*b2[1], b2[2]);
  CheckThetaE();
}


void yugan_a()
{
  a[0] = sin(T[0]) * cos(T[1]);
  a[1] = sin(T[0]) * sin(T[1]);
  a[2] = cos(T[0]);
}

void writeData(){
  std::ofstream plot("ThetaData.dat");
  yugan_a();
  plot << "#THETA_L THETA_E THETA_U THETA_B" << endl;
  for(int i = -45; i <= 120; i++){
    CalTheta[2] = (double)i*M_PI/180.0;
    inverseKinematics();
    plot << -CalTheta[1]-M_PI/2.0 << "\t"
         << -CalTheta[2] << "\t"
         << -CalTheta[3] << "\t"
         << -CalTheta[5] << endl;
  }
}


int main(){

    CalTheta[2] = 91.6732 * M_PI / 180.0;
    inverseKinematics();

  cout << "min = " << min_thetaE*180/(M_PI) << endl;
  cout << "max = " << max_thetaE*180/(M_PI) << endl;
}